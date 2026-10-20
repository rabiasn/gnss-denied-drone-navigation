#!/usr/bin/env python3
"""Receive UAV sensor topics in an existing partition (.venv-gz runtime)."""
import argparse
import json
from pathlib import Path
import threading
import time
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mapnav.png import write_rgb_png
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.camera_info_pb2 import CameraInfo
from google.protobuf.json_format import MessageToDict

ap = argparse.ArgumentParser()
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--seconds", type=float, default=30)
args = ap.parse_args()
if args.out.exists():
    raise FileExistsError("Preserve previous sensor capture; choose a fresh output")
args.out.mkdir(parents=True, exist_ok=True)
node = Node()
frames, imu, truth, calibration = [], [], [], []
lock = threading.Lock()


def stamp(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec


def image_callback(msg):
    with lock:
        if len(frames) >= 4000 or msg.pixel_format_type != 3:
            return
        if msg.step < msg.width * 3 or len(msg.data) < msg.height * msg.step:
            return
        name = f"frame_{len(frames):05d}.png"
        write_rgb_png(args.out / name, msg.width, msg.height, msg.step, msg.data)
        frames.append({"file": name, "sim_stamp_ns": stamp(msg), "arrival_monotonic_ns": time.monotonic_ns(),
                       "width": msg.width, "height": msg.height})


def imu_callback(msg):
    with lock:
        if len(imu) < 150000:
            imu.append({"sim_stamp_ns": stamp(msg), "arrival_monotonic_ns": time.monotonic_ns(),
                "orientation_wxyz": [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
                "angular_velocity_flu": [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                "linear_acceleration_flu": [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]})


def truth_callback(msg):
    with lock:
        if len(truth) < 15000:
            truth.append({"sim_stamp_ns": stamp(msg), "position_enu_m": [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                          "orientation_wxyz": [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z],
                          "linear_velocity_body_flu": [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
                          "angular_velocity_body_flu": [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]})


def calibration_callback(msg):
    with lock:
        if not calibration:
            calibration.append(MessageToDict(msg, preserving_proto_field_name=True))


topics = [(Image, "/mapnav/uav/camera/image", image_callback), (IMU, "/mapnav/uav/imu", imu_callback),
          (Odometry, "/mapnav/evaluation/ground_truth", truth_callback), (CameraInfo, "/mapnav/uav/camera/info", calibration_callback)]
for message_type, topic, callback in topics:
    if not node.subscribe(message_type, topic, callback):
        raise RuntimeError("Cannot register subscription: " + topic)
stop = time.monotonic() + args.seconds
while time.monotonic() < stop:
    time.sleep(.1)
for _, topic, _ in topics:
    node.unsubscribe(topic)
with lock:
    manifest = {"schema_version": 2, "frames": frames.copy(), "imu_count": len(imu), "truth_count": len(truth),
                "camera_calibration_received": bool(calibration), "image_encoding": "lossless RGB PNG",
                "clock_domain": "Gazebo simulation ns", "imu_frame": "body FLU",
                "ground_truth_policy": "Evaluation ONLY; not VIO / localization input",
                "limits": {"images": 4000, "imu": 150000, "truth": 15000}}
    (args.out / "capture.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (args.out / "imu.json").write_text(json.dumps(imu, indent=2) + "\n")
    (args.out / "evaluation_ground_truth.json").write_text(json.dumps(truth, indent=2) + "\n")
    (args.out / "camera_info.json").write_text(json.dumps(calibration, indent=2) + "\n")
print(json.dumps({"frames": len(frames), "imu": len(imu), "truth": len(truth), "calibration": bool(calibration)}))
