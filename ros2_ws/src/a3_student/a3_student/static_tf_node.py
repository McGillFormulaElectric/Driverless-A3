"""A3.1 — transform cones from the `camera` frame into `base_link`.

You are given `/professor/sensor_cones` (geometry_msgs/PoseArray) with
`header.frame_id == "camera"`. The professor is broadcasting a static transform
`base_link -> camera` at (0.5, 0.0, 0.2). Your job:

  1. Subscribe to `/professor/sensor_cones`.
  2. For each pose, look up `base_link <- camera` and transform the pose.
  3. Publish the transformed PoseArray on `<namespace>/cones_base_link` with
     `header.frame_id = "base_link"` and the ORIGINAL header stamp copied
     across (the grader time-matches on it).

Run with your GitHub username as the ROS namespace:

    ros2 launch a3_student static.launch.py github_user:=<your-handle>
"""
import rclpy
import tf2_geometry_msgs  # noqa: F401  (side-effect: registers Pose conversions)
from geometry_msgs.msg import PoseArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

TARGET_FRAME = 'base_link'


class StaticTfNode(Node):
    def __init__(self):
        super().__init__('static_tf_node')

        # tf2 buffer + listener. The listener spins in the background and fills
        # the buffer with every TransformStamped published on /tf and /tf_static.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.sub = self.create_subscription(
            PoseArray, '/professor/sensor_cones', self._on_cones, RELIABLE_QOS
        )
        self.pub = self.create_publisher(PoseArray, 'cones_base_link', RELIABLE_QOS)

        ns = self.get_namespace()
        self.get_logger().info(
            f'Subscribed to /professor/sensor_cones, republishing in {TARGET_FRAME} '
            f'on {ns}/cones_base_link'
        )

    def _on_cones(self, msg: PoseArray) -> None:
        source_frame = msg.header.frame_id  # expect 'camera'
        stamp = Time.from_msg(msg.header.stamp)

        # Look up base_link <- camera at the message stamp. This is a static
        # transform, but using the real stamp is the correct pattern for the
        # real driverless stack.
        try:
            tf = self._tf_buffer.lookup_transform(
                TARGET_FRAME, source_frame, stamp, timeout=Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed ({TARGET_FRAME} <- {source_frame}): {ex}')
            return

        out = PoseArray()
        out.header.stamp = msg.header.stamp   # keep the professor's stamp
        out.header.frame_id = TARGET_FRAME

        # ------------------------------------------------------------------
        # TODO(student): transform every pose in msg.poses into TARGET_FRAME
        # and append it to out.poses.
        #
        # Hint: tf2_geometry_msgs.do_transform_pose(pose, tf) returns the
        # transformed Pose. `tf` is the TransformStamped you just looked up.
        # ------------------------------------------------------------------
        for pose in msg.poses:
            # Replace the next line: do the actual transform.
            transformed_pose = pose
            out.poses.append(transformed_pose)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = StaticTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
