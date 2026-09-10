"""Deterministically synthesize the A3 Neil input bag.

This writes a rosbag2 SQLite bag at `a3_neil/data/inputs.bag/` (by
default) containing 30 seconds of simulated time:

  * `/neil/sensor_cones` (geometry_msgs/PoseArray, @ 20 Hz)
  * `/tf`                (tf2_msgs/TFMessage, @ 50 Hz — map -> base_link)
  * `/tf_static`         (tf2_msgs/TFMessage, once — base_link -> camera)

No ROS graph, no wall clock: message stamps are `t = 0, dt, 2dt, ...` so the
bag is byte-identical between runs and downstream analytic checks
(`scripts/grade_a3.py`) can assume `t0 = 0`.

Run:
    ros2 run a3_neil record_inputs
    ros2 run a3_neil record_inputs --output /tmp/inputs.bag --duration 30.0

Requires the ROS 2 Humble environment (`rosbag2_py`, `rclpy.serialization`).
"""
from __future__ import annotations

import argparse
import math
import os
import shutil

try:
    from rclpy.serialization import serialize_message
    from rosbag2_py import (
        SequentialWriter,
        StorageOptions,
        ConverterOptions,
        TopicMetadata,
    )
    from builtin_interfaces.msg import Time as TimeMsg
    from geometry_msgs.msg import PoseArray, Pose, TransformStamped
    from tf2_msgs.msg import TFMessage
    _ROS_OK = True
except ImportError:
    _ROS_OK = False

from a3_neil.tf_scenario_node import (
    DEFAULT_R,
    DEFAULT_OMEGA,
    DEFAULT_TF_HZ,
    DEFAULT_CONE_HZ,
    DEFAULT_MOUNT_X,
    DEFAULT_MOUNT_Z,
    DEFAULT_CONES_CAMERA_FLAT,
    unflatten_cones,
    yaw_to_quat,
)


def _stamp(t: float):
    """Return a builtin_interfaces/Time at `t` seconds since t0 = 0."""
    sec = int(t)
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    stamp = TimeMsg()
    stamp.sec = sec
    stamp.nanosec = nsec
    return stamp


def _stamp_ns(t: float) -> int:
    """Bag time (int64 nanoseconds) at simulated time t."""
    return int(round(t * 1e9))


def _static_tf(mount_x: float, mount_z: float):
    tf = TransformStamped()
    tf.header.stamp = _stamp(0.0)
    tf.header.frame_id = 'base_link'
    tf.child_frame_id = 'camera'
    tf.transform.translation.x = float(mount_x)
    tf.transform.translation.y = 0.0
    tf.transform.translation.z = float(mount_z)
    tf.transform.rotation.w = 1.0
    return TFMessage(transforms=[tf])


def _dynamic_tf(t: float, R: float, omega: float):
    x = R * math.cos(omega * t)
    y = R * math.sin(omega * t)
    yaw = omega * t + math.pi / 2.0
    qx, qy, qz, qw = yaw_to_quat(yaw)
    tf = TransformStamped()
    tf.header.stamp = _stamp(t)
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'base_link'
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = 0.0
    tf.transform.rotation.x = qx
    tf.transform.rotation.y = qy
    tf.transform.rotation.z = qz
    tf.transform.rotation.w = qw
    return TFMessage(transforms=[tf])


def _cones_msg(t: float, cones):
    msg = PoseArray()
    msg.header.stamp = _stamp(t)
    msg.header.frame_id = 'camera'
    for (cx, cy, cz) in cones:
        p = Pose()
        p.position.x = float(cx)
        p.position.y = float(cy)
        p.position.z = float(cz)
        p.orientation.w = 1.0
        msg.poses.append(p)
    return msg


def write_bag(output: str, duration: float,
              R: float = DEFAULT_R, omega: float = DEFAULT_OMEGA,
              tf_hz: float = DEFAULT_TF_HZ, cone_hz: float = DEFAULT_CONE_HZ,
              mount_x: float = DEFAULT_MOUNT_X, mount_z: float = DEFAULT_MOUNT_Z,
              cones_flat=None) -> None:
    if not _ROS_OK:
        raise SystemExit('Run inside a ROS 2 Humble environment (rosbag2_py not importable).')
    cones = unflatten_cones(list(cones_flat or DEFAULT_CONES_CAMERA_FLAT))

    # rosbag2 refuses to overwrite. Wipe first so re-running is idempotent.
    if os.path.exists(output):
        shutil.rmtree(output)
    os.makedirs(os.path.dirname(output) or '.', exist_ok=True)

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=output, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'),
    )
    writer.create_topic(TopicMetadata(
        name='/tf_static', type='tf2_msgs/msg/TFMessage', serialization_format='cdr'))
    writer.create_topic(TopicMetadata(
        name='/tf', type='tf2_msgs/msg/TFMessage', serialization_format='cdr'))
    writer.create_topic(TopicMetadata(
        name='/neil/sensor_cones', type='geometry_msgs/msg/PoseArray',
        serialization_format='cdr'))

    # /tf_static: once at t=0.
    writer.write('/tf_static',
                 serialize_message(_static_tf(mount_x, mount_z)),
                 _stamp_ns(0.0))

    # Emit /tf @ tf_hz and /neil/sensor_cones @ cone_hz over `duration`.
    tf_dt = 1.0 / tf_hz
    cones_dt = 1.0 / cone_hz
    n_tf = int(round(duration * tf_hz))
    n_cones = int(round(duration * cone_hz))
    for i in range(n_tf):
        t = i * tf_dt
        writer.write('/tf', serialize_message(_dynamic_tf(t, R, omega)), _stamp_ns(t))
    for i in range(n_cones):
        t = i * cones_dt
        writer.write('/neil/sensor_cones',
                     serialize_message(_cones_msg(t, cones)), _stamp_ns(t))

    del writer  # close bag


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default_out = os.path.join(
        os.path.dirname(__file__), '..', 'data', 'inputs.bag'
    )
    parser.add_argument('--output', default=os.path.normpath(default_out),
                        help='Output bag directory (default: a3_neil/data/inputs.bag)')
    parser.add_argument('--duration', type=float, default=30.0,
                        help='Simulated duration in seconds (default: 30.0)')
    args = parser.parse_args()

    write_bag(args.output, args.duration)
    print(f'Wrote bag: {args.output} ({args.duration:.1f} s simulated)')


if __name__ == '__main__':
    main()
