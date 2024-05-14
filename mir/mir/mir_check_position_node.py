import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from mir import mir_api
from math import sqrt
# from action_tutorials_interfaces.action import Fibonacci
# from behavior_tree_ros2_actions.action import FindArucoTag
from behavior_tree_ros2_actions.action import MirMission

import time

class MirPositionNode(Node):

    def __init__(self):
        super().__init__('mir_position_node')
        self.mir = mir_api.MiR()
        self.mir_url = "http://192.168.12.20/api/v2.0.0/"
        self.battery_threshold = 30

        status = self.mir.get_system_info(self.mir_url)
        if status['battery_percentage'] < self.battery_threshold:
            self.get_logger().info('MIR IS LOW ON BATTERY :|')
            self.get_logger().info('CHARGE MIR ROBOT :|')
        else:
            pass
            #self.mir.delete_mission_queue(self.mir_url)

        


        self._action_server = ActionServer(
            self,
            MirMission,
            'mir_check_position',
            self.execute_callback)

        self.mission_to_position = {
            'Workstation3':'Workstation3Rob8',
            'Workstation4':'Workstation4Rob8',
            'Workstation5':'Workstation5Rob8'
        }


        #goal_position = self.mir.get_specific_position(self.mir_url,guid)
        #print(type(goal_position))
        #print(goal_position)


        #goal_pose = []
        #for item in goal_position:
            #print(item)
            #if item['pos_x']:



        #goal_handle.succeed()
        #result = MirMission.Result()
        #result.done = True
    def distance(self,a,b):
        return sqrt((a - b)**2)

    def execute_callback(self, goal_handle):
        
        status = self.mir.get_system_info(self.mir_url)
        if status['battery_percentage'] < self.battery_threshold:
            self.get_logger().info('MIR IS LOW ON BATTERY :|')
            self.get_logger().info('CHARGE MIR ROBOT :|')
        else:
            #self.mir.delete_mission_queue(self.mir_url)
            pass

        if goal_handle.request.mission_id == 'robot_base':
            goal_handle.succeed()
            result = MirMission.Result()
            result.done = True
            return result
        



        test = self.mir.get_position_guid(self.mir_url, self.mission_to_position[goal_handle.request.mission_id])






        mir_pos_x, mir_pos_y, mir_orientation = self.mir.get_current_position(self.mir_url)
        workstation_position = self.mir.get_specific_position(self.mir_url,test)

        workstation_pos_x = workstation_position['pos_x']
        workstation_pos_y = workstation_position['pos_y']
        workstation_pos_orientation = workstation_position['orientation']



        x_dist = self.distance(mir_pos_x,workstation_pos_x)
        y_dist = self.distance(mir_pos_y,workstation_pos_y)
        orientation_dist = self.distance(mir_orientation,workstation_pos_orientation)


        if x_dist < 0.1 and y_dist < 0.1 and orientation_dist < 5:
            goal_handle.succeed()
            result = MirMission.Result()
            result.done = True
            return result
        else:
            goal_handle.abort()
            result = MirMission.Result()
            result.done = False
            return result
        

def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = MirPositionNode()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()