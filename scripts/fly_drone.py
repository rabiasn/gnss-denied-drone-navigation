#!/usr/bin/env python3
"""Open a local Helsinki city, Iris aircraft and isolated ArduPilot SITL.

Only bundled local simulator binaries and loopback UDP are used. This first
flight retains GPS. No physical aircraft, serial ports or safety bypasses.
"""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from mapnav.drone_sim import build_world, check_target, remember_home, flight_timeout_due, VEHICLE_NAME
from mapnav.gazebo_view import check_local_world, gui_config_with_pose, stop_owned_processes, view_commands


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--demo", action="store_true", help="Normal takeoff, short route, return and land with GPS enabled")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--seconds", type=float, default=7200, help="Session wall-clock limit; default 2 hours")
    ap.add_argument("--flight-seconds", type=float, default=180, help="Return to the first Home after this flight duration (30..600 s)")
    ap.add_argument("--exit-after-demo", action="store_true")
    ap.add_argument("--record-sensors", action="store_true", help="Save an 8-second camera/IMU capture during takeoff")
    ap.add_argument("--check", action="store_true", help="Build and validate the world without launching")
    ap.add_argument("--runtime-root", type=Path, default=PROJECT,
                    help="Directory containing vendor/ and build/; default: this repository")
    args = ap.parse_args()
    if not math.isfinite(args.seconds) or not 30 <= args.seconds <= 7200:
        ap.error("Oturum süresi 30..7200 saniye olmalı")
    if not math.isfinite(args.flight_seconds) or not 30 <= args.flight_seconds <= 600:
        ap.error("Uçuş süresi 30..600 saniye olmalı")
    if args.no_gui and not args.check and args.seconds > 300:
        ap.error("Arayüzsüz test için --seconds 30..300 belirt")
    if args.exit_after_demo and not args.demo:
        ap.error("--exit-after-demo, --demo ile kullanılır")
    runtime = args.runtime_root.resolve()
    for p in (runtime / "vendor/ardupilot_gazebo/models/iris_with_standoffs/model.sdf",):
        if not p.is_file():
            ap.error(f"Iris modeli eksik; docs/drone-ucus-kilavuzu.md kurulumunu tamamla: {p}")
    required = [] if args.check else [runtime / "vendor/ardupilot/build/sitl/bin/arducopter", runtime / "build/ardupilot_gazebo/libArduPilotPlugin.dylib"]
    if args.record_sensors:
        required.append(runtime / ".venv-gz/bin/python")
    for p in required:
        if not p.is_file():
            ap.error(f"Yerel simülasyon bileşeni eksik: {p}")
    # A fresh run directory also isolates SITL's eeprom and DataFlash files.
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+"_"+uuid.uuid4().hex[:8]
    out = PROJECT / "artifacts/drone-flight" / run_id
    fdm_port = 19002 if args.check else free_udp_port()
    info = build_world(PROJECT, out, fdm_port, upstream=runtime / "vendor/ardupilot_gazebo")
    preflight = check_local_world(out / "world.sdf")
    checked = subprocess.run(["gz", "sdf", "-k", str(out / "world.sdf")], capture_output=True, text=True, timeout=30)
    if checked.returncode:
        print(checked.stdout+checked.stderr, file=sys.stderr)
        return 1
    if args.check:
        print(json.dumps({**preflight, "sdf_valid": True, "info": info}, indent=2))
        return 0
    from pymavlink import mavutil
    env = os.environ.copy()
    env["GZ_PARTITION"] = "rabia_drone_"+run_id
    env["GZ_IP"] = "127.0.0.1"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(runtime / "build/ardupilot_gazebo")+os.pathsep+env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = str(runtime / "vendor/ardupilot_gazebo/models")+os.pathsep+env.get("GZ_SIM_RESOURCE_PATH", "")
    env["GZ_LOG_PATH"] = str(out / "gzlogs")
    # MAVLink 2 is explicit for this process; no global shell configuration edits.
    os.environ["MAVLINK20"] = "1"
    env["MAVLINK20"] = "1"
    link = mavutil.mavlink_connection("udpin:127.0.0.1:0", source_system=255)
    mav_port = link.port.getsockname()[1]
    x, y = info["launch_xy_m"]
    z = info["pad_top_z_m"]
    gui_config = out / "view.config"
    # Close enough to see a real-size aircraft without scaling it unnaturally.
    gui_config.write_text(gui_config_with_pose(PROJECT / "config/helsinki-view.config", [x-10, y-12, z+8, 0, .39, .86]))
    server, gui = view_commands(out / "world.sdf", gui_config=gui_config)
    lat, lon, elev = info["geodetic_anchor_lat_lon_alt"]
    defaults = [runtime / "vendor/ardupilot/Tools/autotest/default_params" / name for name in ("copter.parm", "gazebo-iris.parm")]
    sitl = [str(runtime / "vendor/ardupilot/build/sitl/bin/arducopter"), "--model", "JSON", "--home", f"{lat},{lon},{elev},0",
            "--sim-port-out", str(fdm_port), "--serial0", f"udpclient:127.0.0.1:{mav_port}",
            "--serial1", "none", "--serial2", "none", "--sysid", "42", "--defaults", ",".join(map(str, defaults))]
    commands = {"server": server, "sitl": sitl}
    if not args.no_gui:
        commands["gui"] = gui
    (out / "session-start.json").write_text(json.dumps({"partition": env["GZ_PARTITION"], "commands": commands, "mavlink_loopback_port": mav_port,
                    "scope": "GPS-enabled simulation only", "launcher_pid": os.getpid()}, indent=2))
    inbox = queue.Queue()
    if sys.stdin.isatty():
        def read_input():
            for line in sys.stdin:
                inbox.put(line.strip())
        threading.Thread(target=read_input, daemon=True).start()
    help_text = ("Komutlar: kalkis | git 5 0 (kuzey, doğu metre) | don (başlangıca dön ve in) | in | durum | cik\n"
                 f"Yükseklik: {info['recommended_altitude_m']:.0f} m. İlk yatay uçuş sınırı: başlangıçtan 10 m. GPS açık.\n"
                 f"Home ilk H noktasında sabit. {args.flight_seconds:.0f} saniye dolunca önce Home'a dönüş, sonra iniş.")
    print(f"Helsinki + Iris + ArduPilot açılıyor.\nKayıt: {out}\n{help_text}", flush=True)
    processes, errors, counts, events = {}, [], {}, []
    began = time.monotonic()
    state = "connecting"
    state_at = began
    heartbeat = None
    local = None
    home_local = None
    takeoff_offset = (0., 0.)
    takeoff_ground_z = None
    timeout_handled = False
    return_settle_ms = None
    return_reason = None
    landing_reason = None
    flight_records = []
    gps_fix, ekf_flags, requested = 0, 0, False
    armed, airborne, landed = False, False, False
    flight_began = None
    last_heartbeat_sent = 0
    last_arm_attempt = 0
    arm_attempts = 0
    altitude = info["recommended_altitude_m"]
    target = None
    reached = []
    demo_pending = args.demo
    demo_phase = 0
    closing = False
    close_at = None
    sensor_started = False
    max_relative_altitude = 0.
    camera_stop = threading.Event()
    camera_follow_result = {"requested": not args.no_gui, "accepted": False}
    old_term = signal.getsignal(signal.SIGTERM)
    old_hup = signal.getsignal(signal.SIGHUP)

    def on_term(signum, frame):
        inbox.put("cik")
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGHUP, on_term)

    def transition(next_state):
        nonlocal state, state_at
        state, state_at = next_state, time.monotonic()
        event = {"state": state, "elapsed_wall_s": round(state_at-began, 3)}
        events.append(event)
        print(f"[{event['elapsed_wall_s']:.1f}s] {state}", flush=True)

    def command(cmd, *params):
        link.mav.command_long_send(42, 1, cmd, 0, *params, *([0.]*(7-len(params))))

    def waypoint(north, east):
        nonlocal target
        n, e, _ = check_target(north, east, altitude)
        if home_local is None:
            raise ValueError("Başlangıç konumu henüz hazır değil")
        target = [home_local[0]+n, home_local[1]+e, home_local[2]-altitude]
        link.mav.set_position_target_local_ned_send(0, 42, 1, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                          3576, *target, 0., 0., 0., 0., 0., 0., 0., 0.)

    def request_land():
        command(mavutil.mavlink.MAV_CMD_NAV_LAND)
        transition("landing")

    def request_return(reason):
        nonlocal return_reason, return_settle_ms
        if state in ("returning", "landing"):
            return
        if home_local is None or not ready:
            errors.append("Home dönüşü için konum kestirimi hazır değil; bulunulan yerde iniş istendi")
            request_land()
            return
        return_reason = reason
        return_settle_ms = None
        waypoint(0, 0)
        transition("returning")
        print(f"İlk H noktasına dönülüyor ({reason}).", flush=True)

    def request_close():
        nonlocal closing, close_at
        if closing:
            return
        closing, close_at = True, time.monotonic()
        if armed:
            request_land()

    def follow_camera():
        # Camera services belong only to our fresh GZ_PARTITION. A separate
        # worker keeps GUI loading/service timeouts out of the control loop.
        for _ in range(35):
            if camera_stop.is_set():
                return
            try:
                followed = subprocess.run(["gz", "service", "-s", "/gui/follow", "--reqtype", "gz.msgs.StringMsg",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "500", "--req", f'data: "{VEHICLE_NAME}"'],
                    env=env, capture_output=True, text=True, timeout=2)
                if "data: true" in followed.stdout:
                    offset = subprocess.run(["gz", "service", "-s", "/gui/follow/offset", "--reqtype", "gz.msgs.Vector3d",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "500", "--req", "x: -6 y: 0 z: 3"],
                        env=env, capture_output=True, text=True, timeout=2)
                    camera_follow_result["accepted"] = "data: true" in offset.stdout
                    if camera_follow_result["accepted"]:
                        print("Kamera drone'u takip ediyor. Serbest bakış için Gazebo'da Esc.", flush=True)
                        return
            except subprocess.TimeoutExpired:
                pass
            camera_stop.wait(1)

    try:
        with ExitStack() as stack:
            telemetry = stack.enter_context((out / "telemetry.jsonl").open("w"))
            for name, cmd in commands.items():
                log = stack.enter_context((out / f"{name}.log").open("w"))
                processes[name] = subprocess.Popen(cmd, env=env, cwd=out, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            camera_thread = threading.Thread(target=follow_camera, daemon=True) if not args.no_gui else None
            if camera_thread:
                camera_thread.start()
            while time.monotonic()-began < args.seconds:
                now = time.monotonic()
                if any(processes[n].poll() is not None for n in ("server", "sitl")):
                    errors.append("Gazebo veya SITL beklenmeden kapandı")
                    break
                if "gui" in processes and processes["gui"].poll() is not None:
                    request_close()
                if closing and (not armed or now-close_at > 45):
                    if not armed and state == "landing":
                        landed = True
                        transition("landed")
                    if armed:
                        errors.append("Kapanışta iniş onayı zaman aşımına uğradı")
                    break
                if now-last_heartbeat_sent >= 1:
                    link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
                    last_heartbeat_sent = now
                msg = link.recv_match(blocking=True, timeout=.1)
                if msg is not None and msg.get_srcSystem() == 42:
                    kind = msg.get_type()
                    counts[kind] = counts.get(kind, 0)+1
                    if kind in ("HEARTBEAT", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "COMMAND_ACK", "STATUSTEXT", "EKF_STATUS_REPORT"):
                        telemetry.write(json.dumps({"elapsed_wall_s": now-began, "message": msg.to_dict()})+"\n")
                    if kind == "HEARTBEAT":
                        heartbeat = msg
                        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        if not requested:
                            link.mav.request_data_stream_send(42, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
                            requested = True
                    elif kind == "LOCAL_POSITION_NED":
                        local = msg
                        if home_local:
                            max_relative_altitude = max(max_relative_altitude, home_local[2]-local.z)
                    elif kind == "GPS_RAW_INT":
                        gps_fix = msg.fix_type
                    elif kind == "EKF_STATUS_REPORT":
                        ekf_flags = msg.flags
                    elif kind == "STATUSTEXT" and ("PreArm" in msg.text or "Arming" in msg.text or "Disarm" in msg.text):
                        print(msg.text, flush=True)
                ready = local is not None and gps_fix >= 3 and bool(ekf_flags & mavutil.mavlink.EKF_POS_HORIZ_ABS)
                if state == "connecting" and ready:
                    home_local = remember_home(home_local, [local.x, local.y, local.z])
                    print("Sabit Home NED:", [round(v, 3) for v in home_local], flush=True)
                    transition("ready")
                    print("Hazır. Kalkış için kalkis yaz veya --demo kullan.", flush=True)
                if demo_pending and state == "ready":
                    inbox.put("kalkis")
                    demo_pending = False
                while not inbox.empty():
                    parts = inbox.get().split()
                    if not parts:
                        continue
                    action = parts[0].lower()
                    try:
                        if action in ("cik", "quit", "exit"):
                            request_close()
                        elif action in ("yardim", "help"):
                            print(help_text, flush=True)
                        elif action in ("durum", "status"):
                            print(f"Durum {state}; GPS fix {gps_fix}; motorlar {'açık' if armed else 'kapalı'}; yerel NED "
                                  f"{[round(local.x,2),round(local.y,2),round(local.z,2)] if local else None}; "
                                  f"Home uzaklığı {math.hypot(local.x-home_local[0],local.y-home_local[1]):.2f} m" if local and home_local else "Konum bekleniyor", flush=True)
                        elif closing:
                            print("Oturum kapanıyor; yeni uçuş komutu kabul edilmedi.", flush=True)
                        elif action in ("kalkis", "takeoff"):
                            if state not in ("ready", "landed") or not ready or armed:
                                raise ValueError("Kalkış için GPS/EKF hazır, motorlar kapalı olmalı")
                            altitude = float(parts[1]) if len(parts) == 2 else info["recommended_altitude_m"]
                            check_target(0, 0, altitude)
                            if altitude < info["recommended_altitude_m"]:
                                raise ValueError(f"Bu başlangıç bölgesinde en az {info['recommended_altitude_m']} m kullan")
                            home_local = remember_home(home_local, [local.x, local.y, local.z])
                            takeoff_offset = (local.x-home_local[0], local.y-home_local[1])
                            check_target(*takeoff_offset, altitude)
                            takeoff_ground_z = local.z
                            flight_began = now
                            timeout_handled = False
                            return_reason = None
                            landing_reason = "manual_land"
                            landed = False
                            demo_phase = 0
                            command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
                            transition("await_guided")
                        elif action in ("git", "goto") and len(parts) == 3:
                            if state != "hover" or not armed:
                                raise ValueError("Önce kalkışı ve havada beklemeyi tamamla")
                            waypoint(float(parts[1]), float(parts[2]))
                            transition("moving")
                        elif action in ("don", "home"):
                            if not armed or state not in ("hover", "moving"):
                                raise ValueError("Dönüş için drone havada olmalı")
                            request_return("don komutu")
                        elif action in ("in", "land"):
                            if armed:
                                request_land()
                            else:
                                print("Motorlar zaten kapalı.", flush=True)
                        else:
                            print(help_text, flush=True)
                    except (ValueError, IndexError) as exc:
                        print(f"Komut uygulanmadı: {exc}", flush=True)
                if closing:
                    continue
                if state == "await_guided" and heartbeat is not None and heartbeat.custom_mode == 4:
                    command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0)
                    arm_attempts, last_arm_attempt = 1, now
                    transition("await_armed")
                elif state == "await_armed":
                    if armed:
                        # ArduPilot takeoff altitude is relative to its current
                        # arming Home; convert our fixed original-Home datum.
                        takeoff_height = altitude + takeoff_ground_z-home_local[2]
                        command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, takeoff_height)
                        transition("taking_off")
                    elif now-last_arm_attempt > 6 and arm_attempts < 4:
                        command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0)
                        arm_attempts += 1
                        last_arm_attempt = now
                elif state == "taking_off" and local is not None and home_local[2]-local.z > altitude-1:
                    airborne = True
                    # A re-takeoff away from H holds its current xy, rather than
                    # silently flying Home or replacing the fixed Home.
                    waypoint(*takeoff_offset)
                    transition("hover")
                elif state in ("moving", "returning") and local is not None and target:
                    distance = math.hypot(local.x-target[0], local.y-target[1])
                    at_target = distance < (0.5 if state == "returning" else 1.) and abs(local.z-target[2]) < 1.5
                    if state == "returning":
                        settled = at_target and math.hypot(local.vx, local.vy) < .4
                        if not settled:
                            return_settle_ms = None
                        elif return_settle_ms is None or local.time_boot_ms < return_settle_ms:
                            return_settle_ms = local.time_boot_ms
                        at_target = settled and return_settle_ms is not None and local.time_boot_ms-return_settle_ms >= 1000
                    if at_target and now-state_at > 3:
                        reached.append({"offset_north_east_m": [target[0]-home_local[0], target[1]-home_local[1]], "error_m": distance})
                        if state == "returning":
                            landing_reason = "home_return"
                            request_land()
                        else:
                            transition("hover")
                elif state == "landing" and heartbeat is not None and not armed:
                    landed = True
                    transition("landed")
                    home_error = math.hypot(local.x-home_local[0], local.y-home_local[1]) if local and home_local else None
                    flight_records.append({"home_ned_m": list(home_local), "takeoff_offset_north_east_m": list(takeoff_offset),
                        "return_reason": return_reason, "landing_reason": landing_reason, "home_error_at_landing_m": home_error})
                    print(f"İniş tamamlandı. İlk H noktasına uzaklık: {home_error:.2f} m.", flush=True)
                    if args.demo and args.exit_after_demo:
                        request_close()
                if args.demo and state == "hover" and now-state_at > 3:
                    if demo_phase == 0:
                        waypoint(6, 0)
                        demo_phase = 1
                        transition("moving")
                    elif demo_phase == 1:
                        waypoint(6, 4)
                        demo_phase = 2
                        transition("moving")
                    elif demo_phase == 2:
                        demo_phase = 3
                        request_return("demo parkuru tamamlandı")
                if args.record_sensors and state == "taking_off" and not sensor_started:
                    log = stack.enter_context((out / "sensors.log").open("w"))
                    processes["sensors"] = subprocess.Popen([str(runtime / ".venv-gz/bin/python"), str(PROJECT / "scripts/record_sensors.py"),
                                       "--out", str(out / "sensors"), "--seconds", "8"], env=env, cwd=PROJECT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    sensor_started = True
                if state in ("await_guided", "await_armed") and now-state_at > 30:
                    errors.append(f"Uçuş hazırlığı zaman aşımı: {state}; güvenlik kontrolü atlanmadı")
                    request_close()
                if flight_began is not None and flight_timeout_due(now-flight_began, args.flight_seconds, armed, state, timeout_handled):
                    timeout_handled = True
                    request_return("uçuş süresi doldu")
                if state == "returning" and now-state_at > 90:
                    errors.append("Home dönüşü zaman aşımı; Home'a varış iddia edilmeden bulunduğu yerde iniş istendi")
                    request_land()
                if state == "connecting" and now-began > 90:
                    errors.append("SITL/GPS/EKF başlangıcı zaman aşımına uğradı")
                    break
                if now-began > args.seconds-45 and armed:
                    request_close()
    except KeyboardInterrupt:
        if armed:
            # Ctrl+C stops only this local simulator; no hardware is connected.
            command(mavutil.mavlink.MAV_CMD_NAV_LAND)
    except (OSError, RuntimeError) as exc:
        errors.append(str(exc))
    finally:
        camera_stop.set()
        stop_owned_processes(list(processes.values()))
        signal.signal(signal.SIGTERM, old_term)
        signal.signal(signal.SIGHUP, old_hup)
        link.close()
        capture_file = out / "sensors/capture.json"
        capture = json.loads(capture_file.read_text()) if capture_file.exists() else None
        result = {"scope": "GPS-enabled local Helsinki/Iris simulation", "partition": env["GZ_PARTITION"],
                  "commands": commands, "flight_events": events, "gps_fix": gps_fix, "mavlink_counts": counts,
                  "airborne": airborne, "landed": landed, "waypoints_reached": reached,
                  "fixed_home_ned_m": list(home_local) if home_local else None, "flights": flight_records,
                  "flight_limit_seconds": args.flight_seconds,
                  "max_relative_altitude_m": max_relative_altitude, "normal_arm_attempts": arm_attempts,
                  "sensor_capture": {"frames": len(capture["frames"]), "imu_samples": capture["imu_count"], "truth_samples": capture["truth_count"]} if capture else None,
                  "camera_follow": camera_follow_result,
                  "errors": errors, "elapsed_wall_s": time.monotonic()-began,
                  "owned_processes_reaped": all(p.poll() is not None for p in processes.values()),
                  "saved_lighting_sha256": info["lighting_world_sha256"],
                  "not_validated": info["not_validated"],
                  "passed": not errors and (not args.demo or airborne and landed and len(reached) >= 3)}
        (out / "result.json").write_text(json.dumps(result, indent=2)+"\n")
        print(json.dumps({k: result[k] for k in ("passed", "airborne", "landed", "errors")}), flush=True)
        print(out, flush=True)
    return int(not result["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
