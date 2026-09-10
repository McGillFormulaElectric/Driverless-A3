"""Neil's TF scenario for A3: a car driving a Formula Student skidpad circle.

Broadcasts:
  * static transform `base_link -> camera` at (mount_x, 0.0, mount_z), no rotation.
  * dynamic transform `map -> base_link` at 50 Hz, with the car pose
    parameterised on a circle of radius R at angular velocity omega:

        x(t) = R * cos(omega * t)
        y(t) = R * sin(omega * t)
        yaw(t) = omega * t + pi/2      (tangent to the circle)

Publishes:
  * `/neil/sensor_cones` (geometry_msgs/PoseArray, frame_id="camera")
    at 20 Hz. Eight fixed cones forming an arc 5-15 m in front of the sensor.

All scenario constants (R, omega, TF/cone rates, camera mount, cone
constellation) are exposed as ROS parameters — see
`a3_neil/config/params.yaml`. Defaults below are the fallback used when
nothing else sets them.
"""
from __future__ import annotations

import math

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from geometry_msgs.msg import PoseArray, Pose, TransformStamped
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    _ROS_OK = True
except ImportError:
    # Allow py_compile / static analysis outside a ROS environment.
    rclpy = None
    Node = object
    _ROS_OK = False


if _ROS_OK:
    RELIABLE_QOS = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )

# --- Fallback defaults (also the ground truth the grader / bag consumers use
# if nothing else has been declared). Keep in sync with config/params.yaml. --
DEFAULT_R = 9.125
DEFAULT_OMEGA = 0.5
DEFAULT_TF_HZ = 50.0
DEFAULT_CONE_HZ = 20.0
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


def yaw_to_quat(yaw: float):
    """Yaw-only quaternion (x, y, z, w)."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def unflatten_cones(flat):
    """Turn a length-3N flat list back into N (x, y, z) tuples."""
    if len(flat) % 3 != 0:
        raise ValueError(f'cones_camera_flat must be a multiple of 3, got {len(flat)}')
    return [(float(flat[i]), float(flat[i + 1]), float(flat[i + 2]))
            for i in range(0, len(flat), 3)]


class TfScenarioNode(Node):
    def __init__(self) -> None:
        super().__init__('tf_scenario_node')

        # --- Parameters (see a3_neil/config/params.yaml) -------------------
        self.declare_parameter('radius', DEFAULT_R)
        self.declare_parameter('omega', DEFAULT_OMEGA)
        self.declare_parameter('tf_hz', DEFAULT_TF_HZ)
        self.declare_parameter('cone_hz', DEFAULT_CONE_HZ)
        self.declare_parameter('mount_x', DEFAULT_MOUNT_X)
        self.declare_parameter('mount_z', DEFAULT_MOUNT_Z)
        self.declare_parameter('cones_camera_flat', DEFAULT_CONES_CAMERA_FLAT)

        self.R = float(self.get_parameter('radius').value)
        self.OMEGA = float(self.get_parameter('omega').value)
        self.TF_HZ = float(self.get_parameter('tf_hz').value)
        self.CONE_HZ = float(self.get_parameter('cone_hz').value)
        self.MOUNT_X = float(self.get_parameter('mount_x').value)
        self.MOUNT_Z = float(self.get_parameter('mount_z').value)
        self.CAMERA_FRAME_CONES = unflatten_cones(
            list(self.get_parameter('cones_camera_flat').value)
        )

        # Static base_link -> camera
        self._static_bc = StaticTransformBroadcaster(self)
        self._publish_static_camera()

        # Dynamic map -> base_link
        self._tf_bc = TransformBroadcaster(self)
        self.create_timer(1.0 / self.TF_HZ, self._tick_tf)

        # /neil/sensor_cones
        self._cones_pub = self.create_publisher(
            PoseArray, '/neil/sensor_cones', RELIABLE_QOS
        )
        self.create_timer(1.0 / self.CONE_HZ, self._tick_cones)

        self._t0 = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            f'TF scenario running: R={self.R} m, omega={self.OMEGA} rad/s '
            f'(~{self.R * self.OMEGA:.2f} m/s), TF@{self.TF_HZ:.0f} Hz, '
            f'cones@{self.CONE_HZ:.0f} Hz'
        )

    # ------------------------------------------------------------------
    # base_link -> camera (static)
    # ------------------------------------------------------------------
    def _publish_static_camera(self) -> None:
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'camera'
        tf.transform.translation.x = self.MOUNT_X
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = self.MOUNT_Z
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
        x = self.R * math.cos(self.OMEGA * t)
        y = self.R * math.sin(self.OMEGA * t)
        yaw = self.OMEGA * t + math.pi / 2.0
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
    # /neil/sensor_cones (PoseArray, 20 Hz)
    # ------------------------------------------------------------------
    def _tick_cones(self) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        for (cx, cy, cz) in self.CAMERA_FRAME_CONES:
            p = Pose()
            p.position.x = float(cx)
            p.position.y = float(cy)
            p.position.z = float(cz)
            p.orientation.w = 1.0  # no rotation on cones
            msg.poses.append(p)
        self._cones_pub.publish(msg)


def main():
    if not _ROS_OK:
        raise SystemExit('Run inside a ROS 2 Humble environment (rclpy not importable).')
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
