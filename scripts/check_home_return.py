#!/usr/bin/env python3
"""Bounded regression: land away, rearm, then timeout-return to original H.

Commands go only to the child simulator's own PTY, never a user Terminal or UAV.
"""
import argparse
import json
import os
from pathlib import Path
import pty
import re
import select
import signal
import subprocess
import sys
import time
import uuid

PROJECT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-root", type=Path, default=PROJECT)
    args = parser.parse_args()
    out = PROJECT / "artifacts/drone-home-check" / uuid.uuid4().hex[:10]
    out.mkdir(parents=True, exist_ok=False)
    master, slave = pty.openpty()
    command = [sys.executable, str(PROJECT / "scripts/fly_drone.py"), "--no-gui", "--seconds", "300", "--flight-seconds", "45"]
    command += ["--runtime-root", str(args.runtime_root.resolve())]
    child = subprocess.Popen(command, stdin=slave, stdout=slave, stderr=slave, cwd=PROJECT, start_new_session=True)
    os.close(slave)
    began = time.monotonic()
    phase, buffer, run_dir = 0, "", None
    sent, transcript = [], []

    def send(value):
        os.write(master, (value+"\n").encode())
        sent.append({"elapsed_wall_s": time.monotonic()-began, "command": value})
        print("REGRESSION COMMAND:", value, flush=True)

    try:
        while time.monotonic()-began < 330 and child.poll() is None:
            if not select.select([master], [], [], 1)[0]:
                continue
            try:
                data = os.read(master, 65536).decode(errors="replace")
            except OSError:
                break
            if not data:
                break
            buffer += data.replace("\r", "")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                transcript.append(line)
                print(line, flush=True)
                if line.startswith("Kayıt: "):
                    run_dir = Path(line.removeprefix("Kayıt: "))
                match = re.fullmatch(r"\[[\d.]+s\] (\w+)", line)
                if not match:
                    continue
                state = match.group(1)
                if phase == 0 and state == "ready":
                    send("kalkis"); phase = 1
                elif phase == 1 and state == "hover":
                    send("git 6 0"); phase = 2
                elif phase == 2 and state == "hover":
                    send("in"); phase = 3
                elif phase == 3 and state == "landed":
                    send("kalkis"); phase = 4
                elif phase == 4 and state == "hover":
                    send("git 6 4"); phase = 5
                elif phase == 5 and state == "hover":
                    print("REGRESSION: Waiting away from H for automatic flight timeout", flush=True)
                    phase = 6
                elif phase == 6 and state == "returning":
                    phase = 7
                elif phase == 7 and state == "landed":
                    send("cik"); phase = 8
        try:
            child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait(timeout=55)
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=55)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
        os.close(master)
        (out / "transcript.txt").write_text("\n".join(transcript)+"\n")
    result_file = run_dir / "result.json" if run_dir else None
    result = json.loads(result_file.read_text()) if result_file and result_file.exists() else {}
    flights = result.get("flights", [])
    checks = {
        "scenario_completed": phase == 8,
        "two_flights": len(flights) == 2,
        "same_original_home": len(flights) == 2 and flights[0]["home_ned_m"] == flights[1]["home_ned_m"],
        "first_landing_away": len(flights) == 2 and flights[0]["home_error_at_landing_m"] > 4,
        "second_takeoff_still_away": len(flights) == 2 and abs(flights[1]["takeoff_offset_north_east_m"][0]) > 4,
        "timeout_caused_return": len(flights) == 2 and flights[1]["return_reason"] == "uçuş süresi doldu",
        "second_landed_at_original_home": len(flights) == 2 and flights[1]["landing_reason"] == "home_return" and flights[1]["home_error_at_landing_m"] < .75,
        "no_errors": result.get("errors") == [],
        "children_reaped": result.get("owned_processes_reaped", False),
    }
    report = {"passed": all(checks.values()), "checks": checks, "flight_records": flights,
              "simulation_result": str(result_file), "commands": sent, "elapsed_wall_s": time.monotonic()-began,
              "scope": "GPS-enabled simulation regression only; not GNSS-denied return"}
    (out / "check.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2), flush=True)
    print(out, flush=True)
    return int(not report["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
