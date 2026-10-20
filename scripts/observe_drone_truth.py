#!/usr/bin/env python3
"""Bounded, read-only Gazebo pose observer for independent evaluation only."""
import argparse
import json
from pathlib import Path
import threading
import time

from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("--seconds", type=float, default=180)
ap.add_argument("--out", type=Path, required=True)
args = ap.parse_args()
if not 1 <= args.seconds <= 300:
    ap.error("Bounded observation requires 1..300 seconds")
if args.out.exists():
    ap.error("Choose a new evaluation output")
records, lock = [], threading.Lock()
node = Node()


def receive(msg):
    with lock:
        if len(records) < 10000:
            records.append({"sim_time_s": msg.header.stamp.sec+msg.header.stamp.nsec/1e9,
                            "xyz_enu_m": [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                            "received_monotonic_s": time.monotonic()})


topic = "/mapnav/evaluation/ground_truth"
if not node.subscribe(Odometry, topic, receive):
    raise RuntimeError("Could not subscribe to isolated evaluation pose")
deadline = time.monotonic()+args.seconds
try:
    while time.monotonic() < deadline:
        time.sleep(.1)
finally:
    node.unsubscribe(topic)
    with lock:
        result = {"scope": "Gazebo pose evaluation ONLY, never estimator input", "samples": records.copy()}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x") as f:
        json.dump(result, f)
    print(json.dumps({"samples": len(result["samples"]), "last": result["samples"][-1] if result["samples"] else None}))
