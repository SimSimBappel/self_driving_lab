import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from mir import mir_api
# from action_tutorials_interfaces.action import Fibonacci
# from behavior_tree_ros2_actions.action import FindArucoTag
from behavior_tree_ros2_actions.action import MirMission

import time

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('mir_server_node')
        self.mir = mir_api.MiR()
        self.mir_url = "http://192.168.100.51/api/v2.0.0/"
        self.battery_threshold = 30

        status = self.mir.get_system_info(self.mir_url)
        if status['battery_percentage'] < self.battery_threshold:
            self.get_logger().info('MIR IS LOW ON BATTERY :|')
            self.get_logger().info('CHARGE MIR ROBOT :|')
        else:
            self.mir.delete_mission_queue(self.mir_url)


        # self.mir.get_mission_done_or_not(mission_id) # to be used with action result

        # self.mir.get_exe_mission() # feedback to actio while running the misson



        self._action_server = ActionServer(
            self,
            MirMission,
            'mir_mission_service',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        
        status = self.mir.get_system_info(self.mir_url)
        if status['battery_percentage'] < self.battery_threshold:
            self.get_logger().info('MIR IS LOW ON BATTERY :|')
            self.get_logger().info('CHARGE MIR ROBOT :|')
            result.done = False
            return result
        else:
            self.mir.delete_mission_queue(self.mir_url)
        
        self.mir.post_to_mission_queue(self.mir_url,goal_handle.request.mission_id)
        while self.mir.get_mission_done_or_not(goal_handle.request.mission_id):
            time.sleep(0.5)
        result = MirMission.Result()
        result.done = True
        return result




def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()