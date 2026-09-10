"""Auto-discovery grader for MFE A3.

Periodically scans the ROS graph for topics matching:
  - /<user>/cones_base_link (geometry_msgs/PoseArray)  -> A3.1
  - /<user>/cones_map       (geometry_msgs/PoseArray)  -> A3.2

For each newly-seen topic it creates a subscription.

Grading strategy:
  * A3.1 has a closed-form ground truth: the static base_link->camera transform
    is a fixed translation (mount_x, 0.0, mount_z), so the expected base_link
    pose of each cone is just its known camera-frame coordinates shifted by
    that vector.
  * A3.2 depends on the moving map->base_link transform. The grader runs its
    own tf2 Buffer + TransformListener and looks up map<-camera at each
    submission's header stamp, so the check is trajectory-agnostic and does
    not require guessing Neil's t0.

Tolerances + scenario geometry are ROS parameters (see
`a3_neil/config/params.yaml`) — the grader is the interactive path; the
bag-based path is scripts/grade_a3.py.

Feedback is published on /neil/feedback (std_msgs/String) as either:
    'Congrats <user>, the answer is correct'
    'Sorry <user>, the answer is incorrect (<metric>)'
The message is only republished when a student transitions between states.
"""
from __future__ import annotations

import math
import re
from collections import deque
from dataclasses import dataclass, field
from typing import Deque

try:
    import numpy as np
    import rclpy
    import tf2_geometry_msgs  # noqa: F401  (side-effect: Pose/PoseStamped conversions)
    from geometry_msgs.msg import PoseArray, PoseStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException
    _ROS_OK = True
except ImportError:
    np = None
    rclpy = None
    Node = object
    TransformException = Exception
    _ROS_OK = False


if _ROS_OK:
    RELIABLE_QOS = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )

# Fallback defaults — real values come from ROS params.
DEFAULT_MOUNT_X = 0.5
DEFAULT_MOUNT_Z = 0.2
DEFAULT_CONES_CAMERA_FLAT = [
    5.0, -1.5, 0.0,
    5.0,  1.5, 0.0,
    8.0, -2.5, 0.0,
    8.0,  2.5, 0.0,
    11.0, -3.0, 0.0,
    11.0,  3.0, 0.0,
    14.0, -3.5, 0.0,
    14.0,  3.5, 0.0,
]
DEFAULT_MATCH_WINDOW = 50
DEFAULT_PART1_TOL_M = 0.05
DEFAULT_PART2_TOL_M = 0.15
DEFAULT_DISCOVERY_PERIOD_S = 2.0
DEFAULT_GRADE_PERIOD_S = 2.0

# Neil owns /neil/*; skip any student submission that would collide.
RESERVED_USERS = {'neil'}

BASE_LINK_RE = re.compile(r'^/([^/]+)/cones_base_link$')
MAP_RE = re.compile(r'^/([^/]+)/cones_map$')


def _unflatten(flat):
    return [(float(flat[i]), float(flat[i + 1]), float(flat[i + 2]))
            for i in range(0, len(flat), 3)]


@dataclass
class GradeState:
    errors: Deque[float] = field(default_factory=deque)
    last_verdict: object = None  # 'correct' | 'incorrect' | None


class Grader(Node):
    def __init__(self) -> None:
        super().__init__('grader')

        # --- Parameters --------------------------------------------------
        self.declare_parameter('part1_tolerance_m', DEFAULT_PART1_TOL_M)
        self.declare_parameter('part2_tolerance_m', DEFAULT_PART2_TOL_M)
        self.declare_parameter('mount_x', DEFAULT_MOUNT_X)
        self.declare_parameter('mount_z', DEFAULT_MOUNT_Z)
        self.declare_parameter('cones_camera_flat', DEFAULT_CONES_CAMERA_FLAT)
        self.declare_parameter('match_window', DEFAULT_MATCH_WINDOW)
        self.declare_parameter('discovery_period_s', DEFAULT_DISCOVERY_PERIOD_S)
        self.declare_parameter('grade_period_s', DEFAULT_GRADE_PERIOD_S)

        self.PART1_TOL_M = float(self.get_parameter('part1_tolerance_m').value)
        self.PART2_TOL_M = float(self.get_parameter('part2_tolerance_m').value)
        self.MOUNT_X = float(self.get_parameter('mount_x').value)
        self.MOUNT_Z = float(self.get_parameter('mount_z').value)
        self.CAMERA_FRAME_CONES = _unflatten(list(
            self.get_parameter('cones_camera_flat').value
        ))
        self.N_CONES = len(self.CAMERA_FRAME_CONES)
        self.MATCH_WINDOW = int(self.get_parameter('match_window').value)
        discovery_s = float(self.get_parameter('discovery_period_s').value)
        grade_s = float(self.get_parameter('grade_period_s').value)

        self.feedback_pub = self.create_publisher(String, '/neil/feedback', RELIABLE_QOS)

        # tf2 buffer/listener used for A3.2 ground-truth (map <- camera).
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Expected base_link cone positions for A3.1 (fixed by the static tf).
        self._expected_base_link = np.array([
            (cx + self.MOUNT_X, cy, cz + self.MOUNT_Z)
            for (cx, cy, cz) in self.CAMERA_FRAME_CONES
        ])

        self._base_states = {}
        self._base_subs = {}
        self._map_states = {}
        self._map_subs = {}

        self.create_timer(discovery_s, self._discover)
        self.create_timer(grade_s, self._grade)

        self.get_logger().info(
            f'Grader running. match_window={self.MATCH_WINDOW}, '
            f'part1_tol={self.PART1_TOL_M} m, part2_tol={self.PART2_TOL_M} m'
        )

    def _new_state(self) -> GradeState:
        return GradeState(errors=deque(maxlen=self.MATCH_WINDOW))

    # --- discovery ---------------------------------------------------------
    def _discover(self) -> None:
        for name, types in self.get_topic_names_and_types():
            if 'geometry_msgs/msg/PoseArray' not in types:
                continue

            m = BASE_LINK_RE.match(name)
            if m:
                user = m.group(1)
                if user in RESERVED_USERS or user in self._base_subs:
                    continue
                self._base_states[user] = self._new_state()
                self._base_subs[user] = self.create_subscription(
                    PoseArray, name, self._make_base_cb(user), RELIABLE_QOS
                )
                self.get_logger().info(f'Discovered A3.1 topic: {name}')
                continue

            m = MAP_RE.match(name)
            if m:
                user = m.group(1)
                if user in RESERVED_USERS or user in self._map_subs:
                    continue
                self._map_states[user] = self._new_state()
                self._map_subs[user] = self.create_subscription(
                    PoseArray, name, self._make_map_cb(user), RELIABLE_QOS
                )
                self.get_logger().info(f'Discovered A3.2 topic: {name}')

    # --- A3.1: base_link check --------------------------------------------
    def _make_base_cb(self, user: str):
        def _cb(msg) -> None:
            if msg.header.frame_id != 'base_link':
                self._base_states[user].errors.append(float('inf'))
                return
            if len(msg.poses) != self.N_CONES:
                self._base_states[user].errors.append(float('inf'))
                return
            got = np.array([(p.position.x, p.position.y, p.position.z) for p in msg.poses])
            per_cone = np.linalg.norm(got - self._expected_base_link, axis=1)
            self._base_states[user].errors.append(float(np.mean(per_cone)))
        return _cb

    # --- A3.2: map check ---------------------------------------------------
    def _make_map_cb(self, user: str):
        def _cb(msg) -> None:
            state = self._map_states[user]
            if msg.header.frame_id != 'map':
                state.errors.append(float('inf'))
                return
            if len(msg.poses) != self.N_CONES:
                state.errors.append(float('inf'))
                return

            stamp = Time.from_msg(msg.header.stamp)
            try:
                tf = self._tf_buffer.lookup_transform(
                    'map', 'camera', stamp, timeout=Duration(seconds=0.2)
                )
            except TransformException:
                return

            expected = np.zeros((self.N_CONES, 3))
            for i, (cx, cy, cz) in enumerate(self.CAMERA_FRAME_CONES):
                ps = PoseStamped()
                ps.header.frame_id = 'camera'
                ps.header.stamp = msg.header.stamp
                ps.pose.position.x = cx
                ps.pose.position.y = cy
                ps.pose.position.z = cz
                ps.pose.orientation.w = 1.0
                out = tf2_geometry_msgs.do_transform_pose(ps.pose, tf)
                expected[i] = (out.position.x, out.position.y, out.position.z)

            got = np.array([(p.position.x, p.position.y, p.position.z) for p in msg.poses])
            per_cone = np.linalg.norm(got - expected, axis=1)
            state.errors.append(float(np.mean(per_cone)))
        return _cb

    # --- scoring -----------------------------------------------------------
    def _grade(self) -> None:
        for user, state in self._base_states.items():
            self._score(user, state, self.PART1_TOL_M, 'base_link')
        for user, state in self._map_states.items():
            self._score(user, state, self.PART2_TOL_M, 'map')

    def _score(self, user: str, state: GradeState, tol: float, label: str) -> None:
        if len(state.errors) < self.MATCH_WINDOW // 2:
            return
        mean_err = float(np.mean(state.errors))
        verdict = 'correct' if math.isfinite(mean_err) and mean_err <= tol else 'incorrect'
        if verdict != state.last_verdict:
            state.last_verdict = verdict
            extra = f'({label} mean_err={mean_err:.3f} m)'
            self._publish_feedback(user, verdict, extra=extra)

    # --- feedback ----------------------------------------------------------
    def _publish_feedback(self, user: str, verdict: str, extra: str = '') -> None:
        if verdict == 'correct':
            text = f'Congrats {user}, the answer is correct'
        else:
            text = f'Sorry {user}, the answer is incorrect'
        if extra:
            text = f'{text} {extra}'
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        self.get_logger().info(text)


def main():
    if not _ROS_OK:
        raise SystemExit('Run inside a ROS 2 Humble environment (rclpy not importable).')
    rclpy.init()
    node = Grader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
