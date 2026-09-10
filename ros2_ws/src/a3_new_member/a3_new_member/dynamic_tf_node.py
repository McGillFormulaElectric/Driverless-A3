"""A3.2 — transform cones from `camera` straight into `map`, through the moving
`base_link` frame.

Neil is broadcasting:
  * static  base_link -> camera   (fixed sensor mount)
  * dynamic map      -> base_link (the car driving a skidpad circle at 50 Hz)

tf2 will chain those for you: ask for `map <- camera` and the buffer walks the
tree. Because the car is moving, the SAME cone in the camera frame produces a
different point in `map` every message. Plot the published PoseArray in
Foxglove with the `map` frame fixed and you should see each cone tracing a
circle — that is the visual proof your dynamic TF chain works.

Publish on `<namespace>/cones_map` with `header.frame_id = "map"` and copy the
original stamp across (the grader time-matches on it).

Run with your GitHub username as the ROS namespace:

    ros2 launch a3_new_member dynamic.launch.py github_user:=<your-handle>
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

TARGET_FRAME = 'map'


class DynamicTfNode(Node):
    def __init__(self):
        super().__init__('dynamic_tf_node')

        # Parameters (see a3_new_member/config/params.yaml).
        self.declare_parameter('tf_lookup_timeout_s', 0.1)
        self.declare_parameter('input_topic', '/neil/sensor_cones')
        self.timeout_s = float(self.get_parameter('tf_lookup_timeout_s').value)
        input_topic = str(self.get_parameter('input_topic').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.sub = self.create_subscription(
            PoseArray, input_topic, self._on_cones, RELIABLE_QOS
        )
        self.pub = self.create_publisher(PoseArray, 'cones_map', RELIABLE_QOS)

        ns = self.get_namespace()
        self.get_logger().info(
            f'Subscribed to {input_topic}, republishing in {TARGET_FRAME} '
            f'on {ns}/cones_map'
        )

    def _on_cones(self, msg) -> None:
        source_frame = msg.header.frame_id  # expect 'camera'
        stamp = Time.from_msg(msg.header.stamp)

        # tf2 will chain map <- base_link <- camera automatically.
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
        # transformed Pose. The same call works for chained transforms —
        # tf2 has already composed `map <- camera` for you.
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
    node = DynamicTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
