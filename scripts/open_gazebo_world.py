#!/usr/bin/env python3
"""Open the packaged Helsinki world: separate server / GUI, one command.

The fresh partition prevents accidental connections to other Gazebo sessions.
Closing the GUI or pressing Ctrl+C stops only this launcher's children.
Saved scene and light values are never rewritten by this launcher.
"""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from mapnav.gazebo_view import check_local_world, stop_owned_processes, view_commands, gui_config_with_pose


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--view", choices=("overview", "district", "roof", "top"), default="district", help="GUI starting viewpoint")
    ap.add_argument("--world", type=Path, default=PROJECT / "gazebo/helsinki/world.sdf", help="Explicit local world")
    ap.add_argument("--backend", choices=("metal", "opengl"), default="metal" if sys.platform == "darwin" else "opengl")
    ap.add_argument("--gui-config", type=Path, default=PROJECT / "gazebo/helsinki/view.config", help="Local GUI configuration")
    ap.add_argument("--check", action="store_true", help="Validate local files and SDF; do not launch GUI / server")
    ap.add_argument("--no-gui", action="store_true", help="Server-only diagnostic")
    ap.add_argument("--seconds", type=float, help="Bounded diagnostic; otherwise runs until GUI closes / Ctrl+C")
    args = ap.parse_args()
    if args.seconds is not None and (not math.isfinite(args.seconds) or not 0 < args.seconds <= 300):
        ap.error("--seconds must be finite and in (0, 300]")
    executable = shutil.which("gz")
    if executable is None:
        ap.error("gz bulunamadı; Gazebo kurulumunun PATH içinde olması gerekiyor")
    try:
        preflight = check_local_world(args.world)
        if not args.no_gui and not args.gui_config.is_file():
            raise FileNotFoundError(f"Missing GUI configuration: {args.gui_config}")
    except (OSError, ValueError) as exc:
        ap.error(str(exc) + "\nModel paketi için: python3 scripts/download_gazebo_assets.py")
    validation = subprocess.run([executable, "sdf", "-k", str(args.world.resolve())],
                                capture_output=True, text=True, timeout=20)
    if validation.returncode != 0:
        print(validation.stdout + validation.stderr, file=sys.stderr)
        return 1
    if args.check:
        print(json.dumps({**preflight, "sdf_valid": True, "launched": False}, indent=2))
        return 0
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:10]
    out = PROJECT / ".gazebo-logs" / run_id
    out.mkdir(parents=True, exist_ok=False)
    if args.view is not None:
        pose = json.loads((PROJECT / "config/helsinki_views.json").read_text())[args.view]
        configured = gui_config_with_pose(args.gui_config, pose)
        args.gui_config = out / "view.config"
        args.gui_config.write_text(configured)
    env = os.environ.copy()
    env["GZ_PARTITION"] = "rabia_helsinki_" + run_id
    env["GZ_IP"] = "127.0.0.1"
    env["GZ_LOG_PATH"] = str(out / "logs")
    env["GZ_SIM_RESOURCE_PATH"] = str(args.world.resolve().parent) + os.pathsep + env.get("GZ_SIM_RESOURCE_PATH", "")
    server_command, gui_command = view_commands(args.world, args.backend, executable, args.gui_config)
    commands, processes, errors = [server_command], [], []
    reason = "unknown"
    began = time.monotonic()
    old_term_handler = signal.getsignal(signal.SIGTERM)

    def on_term(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)
    print(f"Dünya: {args.world.resolve()}", flush=True)
    print(f"Kayıtlar: {out}\nKapatmak için Ctrl+C; yalnız bu oturum kapanır.", flush=True)
    try:
        with ExitStack() as stack:
            server_log = stack.enter_context((out / "server.log").open("w"))
            server = subprocess.Popen(server_command, env=env, stdout=server_log,
                                      stderr=subprocess.STDOUT, start_new_session=True,
                                      cwd=args.world.resolve().parent)
            processes.append(server)
            # Gazebo GUI reconnects while the server finishes loading models.
            # A short initial delay also catches immediate server-start failures.
            time.sleep(1)
            if server.poll() is not None:
                errors.append(f"Server exited during startup: {server.returncode}")
                reason = "server_start_failed"
            elif not args.no_gui:
                gui_log = stack.enter_context((out / "gui.log").open("w"))
                gui = subprocess.Popen(gui_command, env=env, stdout=gui_log,
                                       stderr=subprocess.STDOUT, start_new_session=True,
                                       cwd=args.world.resolve().parent)
                processes.append(gui)
                commands.append(gui_command)
            while not errors:
                if server.poll() is not None:
                    errors.append(f"Server exited before requested shutdown: {server.returncode}")
                    reason = "server_exited"
                    break
                if len(processes) > 1 and processes[1].poll() is not None:
                    reason = "gui_closed"
                    if processes[1].returncode != 0:
                        errors.append(f"GUI exited with status {processes[1].returncode}")
                    break
                if args.seconds is not None and time.monotonic() - began >= args.seconds:
                    reason = "bounded_duration_completed"
                    break
                time.sleep(.2)
    except KeyboardInterrupt:
        reason = "user_stopped"
    except OSError as exc:
        reason = "launch_error"
        errors.append(str(exc))
    finally:
        stop_owned_processes(processes)
        signal.signal(signal.SIGTERM, old_term_handler)
        result = {"schema_version": 1, **preflight, "partition": env["GZ_PARTITION"],
                  "commands": commands, "gui_requested": not args.no_gui,
                  "elapsed_wall_seconds": time.monotonic() - began, "stop_reason": reason,
                  "errors": errors, "owned_processes_reaped": all(p.poll() is not None for p in processes),
                  "scope": "Local world viewer only; no aircraft / SITL / existing session control"}
        (out / "session.json").write_text(json.dumps(result, indent=2) + "\n")
    if errors:
        print("\n".join(errors) + f"\nAyrıntı: {out}", file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
