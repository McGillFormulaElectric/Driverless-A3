"""Professor's TF scenario for A3: a car driving a Formula Student skidpad circle.

Broadcasts:
  * static transform `base_link -> camera` at (0.5, 0.0, 0.2), no rotation.
  * dynamic transform `map -> base_link` at 50 Hz, with the car pose
    parameterised on a circle of radius R at angular velocity omega:

        x(t) = R * cos(omega * t)
        y(t) = R * sin(omega * t)
        yaw(t) = omega * t + pi/2      (tangent to the circle)

Publishes:
  * `/professor/sensor_cones` (geometry_msgs/PoseArray, frame_id="camera")
    at 20 Hz. Eight fixed cones forming an arc 5-15 m in front of the sensor.

Students then transform those cones through `base_link` and `map` to prove
their TF chain works.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# --- Skidpad geometry (matches Formula Student inner-line ~9.125 m radius) ---
R = 9.125          # metres
OMEGA = 0.5        # rad/s -> tangential speed ~ R*OMEGA ~ 4.6 m/s
TF_HZ = 50.0       # dynamic map->base_link broadcast rate
CONES_HZ = 20.0    # sensor publication rate

# --- Fixed camera mount on the car (base_link -> camera) ---
CAMERA_XYZ = (0.5, 0.0, 0.2)  # 0.5 m in front of base_link, 0.2 m up.

# --- Cone constellation in the camera frame -------------------------------
# Eight cones forming an arc 5-15 m in front of the sensor, spread laterally.
# These are the ground-truth positions the grader will compare against after
# the student transforms them.
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


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    """Yaw-only quaternion (x, y, z, w)."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class TfScenarioNode(Node):
    def __init__(self) -> None:
        super().__init__('tf_scenario_node')

        # Static base_link -> camera
        self._static_bc = StaticTransformBroadcaster(self)
        self._publish_static_camera()

        # Dynamic map -> base_link
        self._tf_bc = TransformBroadcaster(self)
        self.create_timer(1.0 / TF_HZ, self._tick_tf)

        # /professor/sensor_cones
        self._cones_pub = self.create_publisher(
            PoseArray, '/professor/sensor_cones', RELIABLE_QOS
        )
        self.create_timer(1.0 / CONES_HZ, self._tick_cones)

        self._t0 = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            f'TF scenario running: R={R} m, omega={OMEGA} rad/s '
            f'(~{R * OMEGA:.2f} m/s), TF@{TF_HZ:.0f} Hz, cones@{CONES_HZ:.0f} Hz'
        )

    # ------------------------------------------------------------------
    # base_link -> camera (static)
    # ------------------------------------------------------------------
    def _publish_static_camera(self) -> None:
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'camera'
        tf.transform.translation.x = CAMERA_XYZ[0]
        tf.transform.translation.y = CAMERA_XYZ[1]
        tf.transform.translation.z = CAMERA_XYZ[2]
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self._static_bc.sendTransform(tf)

    # ------------------------------------------------------------------
    # map -> base_link (dynamic, 50 Hz)
    # ------------------------------------------------------------------
    def _tick_tf(self) -> None:
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9 - self._t0
        x = R * math.cos(OMEGA * t)
        y = R * math.sin(OMEGA * t)
        yaw = OMEGA * t + math.pi / 2.0
        qx, qy, qz, qw = yaw_to_quat(yaw)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_bc.sendTransform(tf)

    # ------------------------------------------------------------------
    # /professor/sensor_cones (PoseArray, 20 Hz)
    # ------------------------------------------------------------------
    def _tick_cones(self) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        for (cx, cy, cz) in CAMERA_FRAME_CONES:
            p = Pose()
            p.position.x = float(cx)
            p.position.y = float(cy)
            p.position.z = float(cz)
            p.orientation.w = 1.0  # no rotation on cones
            msg.poses.append(p)
        self._cones_pub.publish(msg)


def main():
    rclpy.init()
    node = TfScenarioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
