"""Auto-discovery grader for MFE A3.

Periodically scans the ROS graph for topics matching:
  - /<user>/cones_base_link (geometry_msgs/PoseArray)  -> A3.1
  - /<user>/cones_map       (geometry_msgs/PoseArray)  -> A3.2

For each newly-seen topic it creates a subscription.

Grading strategy:
  * A3.1 has a closed-form ground truth: the static base_link->camera transform
    is a fixed translation (0.5, 0.0, 0.2), so the expected base_link pose of
    each cone is just its known camera-frame coordinates shifted by that vector.
  * A3.2 depends on the moving map->base_link transform. The grader runs its
    own tf2 Buffer + TransformListener and looks up map<-camera at each
    student message's header stamp, so the check is trajectory-agnostic and
    does not require guessing the professor's t0.

Per-cone position tolerances (mean over the last ~50 samples):
  * A3.1: <= 0.05 m
  * A3.2: <= 0.15 m

Feedback is published on /professor/feedback (std_msgs/String) as either:
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

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (side-effect: registers Pose/PoseStamped conversions)
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Must match the professor tf_scenario_node — duplicated intentionally so the
# grader does not import the professor node module.
CAMERA_XYZ = (0.5, 0.0, 0.2)
CAMERA_FRAME_CONES = [
    (5.0, -1.5, 0.0),
    (5.0,  1.5, 0.0),
    (8.0, -2.5, 0.0),
    (8.0,  2.5, 0.0),
    (11.0, -3.0, 0.0),
    (11.0,  3.0, 0.0),
    (14.0, -3.5, 0.0),
    (14.0,  3.5, 0.0),
]
N_CONES = len(CAMERA_FRAME_CONES)

WINDOW = 50                 # samples per student used to score
BASE_LINK_TOL_M = 0.05      # A3.1 mean per-cone error tolerance
MAP_TOL_M = 0.15            # A3.2 mean per-cone error tolerance
DISCOVERY_PERIOD_S = 2.0
GRADE_PERIOD_S = 2.0

BASE_LINK_RE = re.compile(r'^/([^/]+)/cones_base_link$')
MAP_RE = re.compile(r'^/([^/]+)/cones_map$')


def _reserved(user: str) -> bool:
    """Skip the professor's own namespaces (and any future professor_* helpers)."""
    return user == 'professor' or user.startswith('professor_')


@dataclass
class GradeState:
    errors: Deque[float] = field(default_factory=lambda: deque(maxlen=WINDOW))
    last_verdict: str | None = None  # 'correct' | 'incorrect' | None


class Grader(Node):
    def __init__(self) -> None:
        super().__init__('grader')

        self.feedback_pub = self.create_publisher(String, '/professor/feedback', RELIABLE_QOS)

        # tf2 buffer/listener used for A3.2 ground-truth (map <- camera).
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Expected base_link cone positions for A3.1 (fixed by the static tf).
        self._expected_base_link = np.array([
            (cx + CAMERA_XYZ[0], cy + CAMERA_XYZ[1], cz + CAMERA_XYZ[2])
            for (cx, cy, cz) in CAMERA_FRAME_CONES
        ])

        self._base_states: dict[str, GradeState] = {}
        self._base_subs: dict[str, object] = {}
        self._map_states: dict[str, GradeState] = {}
        self._map_subs: dict[str, object] = {}

        self.create_timer(DISCOVERY_PERIOD_S, self._discover)
        self.create_timer(GRADE_PERIOD_S, self._grade)

        self.get_logger().info(
            f'Grader running. window={WINDOW}, '
            f'base_link_tol={BASE_LINK_TOL_M} m, map_tol={MAP_TOL_M} m'
        )

    # --- discovery ---------------------------------------------------------
    def _discover(self) -> None:
        for name, types in self.get_topic_names_and_types():
            if 'geometry_msgs/msg/PoseArray' not in types:
                continue

            m = BASE_LINK_RE.match(name)
            if m:
                user = m.group(1)
                if _reserved(user) or user in self._base_subs:
                    continue
                self._base_states[user] = GradeState()
                self._base_subs[user] = self.create_subscription(
                    PoseArray, name, self._make_base_cb(user), RELIABLE_QOS
                )
                self.get_logger().info(f'Discovered A3.1 topic: {name}')
                continue

            m = MAP_RE.match(name)
            if m:
                user = m.group(1)
                if _reserved(user) or user in self._map_subs:
                    continue
                self._map_states[user] = GradeState()
                self._map_subs[user] = self.create_subscription(
                    PoseArray, name, self._make_map_cb(user), RELIABLE_QOS
                )
                self.get_logger().info(f'Discovered A3.2 topic: {name}')

    # --- A3.1: base_link check --------------------------------------------
    def _make_base_cb(self, user: str):
        def _cb(msg: PoseArray) -> None:
            if msg.header.frame_id != 'base_link':
                # Wrong frame is an automatic fail signal for this sample.
                self._base_states[user].errors.append(float('inf'))
                return
            if len(msg.poses) != N_CONES:
                self._base_states[user].errors.append(float('inf'))
                return
            got = np.array([(p.position.x, p.position.y, p.position.z) for p in msg.poses])
            per_cone = np.linalg.norm(got - self._expected_base_link, axis=1)
            self._base_states[user].errors.append(float(np.mean(per_cone)))
        return _cb

    # --- A3.2: map check ---------------------------------------------------
    def _make_map_cb(self, user: str):
        def _cb(msg: PoseArray) -> None:
            state = self._map_states[user]
            if msg.header.frame_id != 'map':
                state.errors.append(float('inf'))
                return
            if len(msg.poses) != N_CONES:
                state.errors.append(float('inf'))
                return

            # Ground truth: transform the known camera-frame cones into map at
            # the student message timestamp (using our own tf listener).
            stamp = Time.from_msg(msg.header.stamp)
            try:
                tf = self._tf_buffer.lookup_transform(
                    'map', 'camera', stamp, timeout=Duration(seconds=0.2)
                )
            except TransformException:
                # No transform yet or too old — skip this sample rather than fail.
                return

            expected = np.zeros((N_CONES, 3))
            for i, (cx, cy, cz) in enumerate(CAMERA_FRAME_CONES):
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
            self._score(user, state, BASE_LINK_TOL_M, 'base_link')
        for user, state in self._map_states.items():
            self._score(user, state, MAP_TOL_M, 'map')

    def _score(self, user: str, state: GradeState, tol: float, label: str) -> None:
        if len(state.errors) < WINDOW // 2:
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
