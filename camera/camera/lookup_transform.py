import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from behavior_tree_ros2_actions.srv import LookupTransform
from geometry_msgs.msg import TransformStamped, PoseStamped


from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile, HistoryPolicy
class FrameListener(Node):

    def __init__(self):
        super().__init__('_tf2_frame_listener')
        self.srv = self.create_service(LookupTransform, 'lookup_transform', self.lookup_transform)

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        _qos = QoSProfile(
                depth=100000,#1000000
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True, qos=_qos, static_qos=_qos)

    def lookup_transform(self, request, response):
        from_frame_rel = request.source 
        to_frame_rel = request.target

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            print(t)
            response.result = True
            # response.transform = t
            response.transform = PoseStamped()
            response.transform.header.stamp = self.get_clock().now().to_msg()
            response.transform.header.frame_id = "panda_link0"
            response.transform.pose.position.x = t.transform.translation.x
            response.transform.pose.position.y = t.transform.translation.y
            response.transform.pose.position.z = t.transform.translation.z
            response.transform.pose.orientation.x = t.transform.rotation.x
            response.transform.pose.orientation.y = t.transform.rotation.y
            response.transform.pose.orientation.z = t.transform.rotation.z
            response.transform.pose.orientation.w = t.transform.rotation.w
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            response.result = False
            # response.transform = TransformStamped()
            response.transform = PoseStamped()
        
        
        return response


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
        print("dead?")
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()