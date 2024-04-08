import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from behavior_tree_ros2_actions import LookupTransform
from geometry_msgs.msg import TransformStamped



class FrameListener(Node):

    def __init__(self):
        super().__init__('_tf2_frame_listener')
        self.srv = self.create_service(LookupTransform, 'add_two_ints', self.lookup_transform)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(5.0, self.on_timer)

    def lookup_transform(self, request, response):
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        from_frame_rel = request.source 
        to_frame_rel = request.target

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            response.result = True
            response.transform = t
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            response.result = False
            response.transform = TransformStamped()
        
        
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