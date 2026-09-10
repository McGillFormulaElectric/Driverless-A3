#!/usr/bin/env python3
"""Offline grader for MFE A3.

Usage:
    python3 scripts/grade_a3.py --answer-bag submissions/<user>_a3_1.bag --part 1
    python3 scripts/grade_a3.py --answer-bag submissions/<user>_a3_2.bag --part 2

Exits 0 (PASS) or 1 (FAIL). Run inside a ROS 2 Humble environment.
"""
import argparse
import math
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseArray
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

import numpy as np

# Scenario constants (must match a3_neil/config/params.yaml)
RADIUS = 9.125       # m
OMEGA = 0.5          # rad/s
MOUNT_X = 0.5        # camera x in base_link
MOUNT_Z = 0.2        # camera z in base_link
PART1_TOL = 0.05     # m
PART2_TOL = 0.15     # m


def rotation_z(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def camera_to_base(pt):
    """camera frame → base_link via static mount (translation only for our mount)."""
    return np.array([pt[0] + MOUNT_X, pt[1], pt[2] + MOUNT_Z])


def base_to_map(pt, t):
    """base_link → map using analytic circular trajectory at time t."""
    theta = OMEGA * t + math.pi / 2
    R = rotation_z(theta)
    x_car = RADIUS * math.cos(OMEGA * t)
    y_car = RADIUS * math.sin(OMEGA * t)
    return R @ pt + np.array([x_car, y_car, 0.0])


def open_reader(bag_path):
    if not ROS_AVAILABLE:
        sys.exit("Run inside a ROS 2 Humble environment (rosbag2_py not importable).")
    storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv_opts = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, conv_opts)
    return reader


def read_pose_arrays(bag_path, topic):
    reader = open_reader(bag_path)
    topic_types = {m.name: m.type for m in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        print(f"[FAIL] Topic '{topic}' not found in bag. Found: {list(topic_types.keys())}")
        return []
    msgs = []
    while reader.has_next():
        name, data, ts = reader.read_next()
        if name == topic:
            msg = deserialize_message(data, PoseArray)
            msgs.append((ts * 1e-9, msg))
    return msgs


def grade(answer_bag, part, input_bag=None):
    # Discover the user from the bag topic list
    reader = open_reader(answer_bag)
    topic_types = {m.name: m.type for m in reader.get_all_topics_and_types()}
    del reader

    suffix = "cones_base_link" if part == 1 else "cones_map"
    matches = [t for t in topic_types if t.endswith(f"/{suffix}")]
    if not matches:
        print(f"[FAIL] No topic ending in '/{suffix}' found in {answer_bag}")
        sys.exit(1)
    answer_topic = matches[0]
    user = answer_topic.split("/")[1]
    print(f"Grading user '{user}', part {part}, topic '{answer_topic}'")

    answer_msgs = read_pose_arrays(answer_bag, answer_topic)
    if not answer_msgs:
        print("[FAIL] No messages found on answer topic.")
        sys.exit(1)

    errors = []
    for ts, pa in answer_msgs:
        if not pa.poses:
            continue
        for pose in pa.poses:
            pt_ans = np.array([pose.position.x, pose.position.y, pose.position.z])
            # We don't have the original camera-frame cones, but we can check that
            # the output is a reasonable distance from the car origin.
            # For Part 1: cones should be in base_link — x positive, small y.
            # For Part 2: cones should be in map — plausible world coords.
            # Real grading: compare to expected from the input bag (if provided).
            if input_bag:
                # Full analytic comparison would go here with the input bag.
                pass
            errors.append(0.0)  # placeholder if no input bag

    if input_bag:
        print("[INFO] Input bag provided — using analytic comparison.")
        # Read sensor_cones from input bag and compare analytically
        sensor_msgs = read_pose_arrays(input_bag, "/neil/sensor_cones")
        answer_msgs2 = read_pose_arrays(answer_bag, answer_topic)
        if not sensor_msgs or not answer_msgs2:
            print("[FAIL] Could not read sensor or answer messages.")
            sys.exit(1)
        sensor_ts = np.array([t for t, _ in sensor_msgs])
        all_errors = []
        for ts, pa in answer_msgs2:
            idx = int(np.argmin(np.abs(sensor_ts - ts)))
            _, sensor_pa = sensor_msgs[idx]
            for i, pose in enumerate(pa.poses):
                if i >= len(sensor_pa.poses):
                    break
                sp = sensor_pa.poses[i].position
                cam_pt = np.array([sp.x, sp.y, sp.z])
                if part == 1:
                    expected = camera_to_base(cam_pt)
                else:
                    bl_pt = camera_to_base(cam_pt)
                    expected = base_to_map(bl_pt, sensor_ts[idx])
                ans_pt = np.array([pose.position.x, pose.position.y, pose.position.z])
                all_errors.append(np.linalg.norm(ans_pt - expected))
        if not all_errors:
            print("[FAIL] No cone comparisons could be made.")
            sys.exit(1)
        mean_err = float(np.mean(all_errors))
        tol = PART1_TOL if part == 1 else PART2_TOL
        print(f"Mean per-cone position error: {mean_err:.4f} m  (tolerance: {tol} m)")
        if mean_err <= tol:
            print(f"PASS")
            sys.exit(0)
        else:
            print(f"FAIL")
            sys.exit(1)
    else:
        print("[INFO] No input bag provided — basic sanity check only.")
        print(f"Found {len(answer_msgs)} messages with poses. PASS (sanity only).")
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description="MFE A3 offline grader")
    parser.add_argument("--answer-bag", required=True, help="Path to student answer bag dir")
    parser.add_argument("--part", type=int, required=True, choices=[1, 2])
    parser.add_argument("--input-bag", default=None, help="Path to input bag dir")
    args = parser.parse_args()
    grade(args.answer_bag, args.part, args.input_bag)


if __name__ == "__main__":
    main()
