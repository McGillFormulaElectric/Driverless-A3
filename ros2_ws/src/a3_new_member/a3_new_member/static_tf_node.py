"""A3.1 — transform cones from the `camera` frame into `base_link`.

You are given `/neil/sensor_cones` (geometry_msgs/PoseArray) with
`header.frame_id == "camera"`. Neil is broadcasting a static transform
`base_link -> camera` at (0.5, 0.0, 0.2). Your job:

  1. Subscribe to `/neil/sensor_cones` (topic is a ROS param — override
     it if you're playing back a bag on a different name).
  2. For each pose, look up `base_link <- camera` and transform the pose.
  3. Publish the transformed PoseArray on `<namespace>/cones_base_link` with
     `header.frame_id = "base_link"` and the ORIGINAL header stamp copied
     across (the grader time-matches on it).

Run with your GitHub username as the ROS namespace:

    ros2 launch a3_new_member static.launch.py github_user:=<your-handle>
"""
try:
    import rclpy
    import tf2_geometry_msgs  # noqa: F401  (side-effect: registers Pose conversions)
    from geometry_msgs.msg import PoseArray
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    _ROS_OK = True
except ImportError:
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

TARGET_FRAME = 'base_link'


class StaticTfNode(Node):
    def __init__(self):
        super().__init__('static_tf_node')

        # Parameters (see a3_new_member/config/params.yaml).
        self.declare_parameter('tf_lookup_timeout_s', 0.1)
        self.declare_parameter('input_topic', '/neil/sensor_cones')
        self.timeout_s = float(self.get_parameter('tf_lookup_timeout_s').value)
        input_topic = str(self.get_parameter('input_topic').value)

        # tf2 buffer + listener. The listener spins in the background and fills
        # the buffer with every TransformStamped published on /tf and /tf_static.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.sub = self.create_subscription(
            PoseArray, input_topic, self._on_cones, RELIABLE_QOS
        )
        self.pub = self.create_publisher(PoseArray, 'cones_base_link', RELIABLE_QOS)

        ns = self.get_namespace()
        self.get_logger().info(
            f'Subscribed to {input_topic}, republishing in {TARGET_FRAME} '
            f'on {ns}/cones_base_link'
        )

    def _on_cones(self, msg) -> None:
        source_frame = msg.header.frame_id  # expect 'camera'
        stamp = Time.from_msg(msg.header.stamp)

        # Look up base_link <- camera at the message stamp. This is a static
        # transform, but using the real stamp is the correct pattern for the
        # real driverless stack.
        try:
            tf = self._tf_buffer.lookup_transform(
                TARGET_FRAME, source_frame, stamp, timeout=Duration(seconds=self.timeout_s)
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed ({TARGET_FRAME} <- {source_frame}): {ex}')
            return

        out = PoseArray()
        out.header.stamp = msg.header.stamp   # keep Neil's stamp
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
    if not _ROS_OK:
        raise SystemExit('Run inside a ROS 2 Humble environment (rclpy not importable).')
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
