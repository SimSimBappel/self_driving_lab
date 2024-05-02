import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from behavior_tree_ros2_actions.srv import LookupTransform
from geometry_msgs.msg import PoseStamped

class FrameListener(Node):

    def __init__(self):
        super().__init__('_tf2_frame_listener')
        self.srv = self.create_service(LookupTransform, 'lookup_transform', self.lookup_transform)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

    def lookup_transform(self, request, response):
        try:
            t = self.tf_buffer.lookup_transform(
                request.target,
                request.source,
                rclpy.time.Time())
            
            # Create response
            response.result = True
            response.transform = PoseStamped()
            response.transform.header.stamp = self.get_clock().now().to_msg()
            response.transform.header.frame_id = "panda_link0"
            response.transform.pose.position = t.transform.translation
            response.transform.pose.orientation = t.transform.rotation

        except TransformException as ex:
            self.get_logger().info(f'Could not transform {request.target} to {request.source}: {ex}')
            response.result = False
            response.transform = PoseStamped()

            self.print_frames()
        
        return response
    
    def print_frames(self):
        # Output all frames in the tf2 tree
        frames = self.tf_buffer.all_frames_as_string()
        self.get_logger().info('Current frames in tf2 tree:')
        for frame in frames.split('\n'):
            self.get_logger().info(f'- {frame.strip()}')


def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print('Lookup_transform Shutting down...')
    rclpy.shutdown()


if __name__ == '__main__':
    main()