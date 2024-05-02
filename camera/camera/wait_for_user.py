import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from behavior_tree_ros2_actions.action import WaitForUser


class UserActionServer(Node):

    def __init__(self):
        super().__init__('user_action_server')
        self._action_server = ActionServer(self, WaitForUser, 'wait_for_user', self.execute_callback)
        self.get_logger().info('User action server initialized.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('waiting...')

        input("Press Enter to continue...")

        self.get_logger().info('Proceeding...')
        goal_handle.succeed()
        
        return WaitForUser.Result(done=True)


def main(args=None):
    rclpy.init(args=args)
    user_action_server = UserActionServer()
    rclpy.spin(user_action_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
