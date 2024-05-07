import xml.etree.ElementTree as ET
from xdl_parser_interface.srv import XdlConversion
from behavior_tree_ros2_actions.action import Xdl
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
import tkinter as tk
from tkinter import filedialog
import rclpy
import os
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import time
# sussy = '/home/lilholt/github/sdl_ws/src/self_driving_lab/xdl_parser2/xdl_parser2/test.xml'
#<SubTree ID="Add"/>
sussy = '/home/intelnuc/sdl_ws/src/self_driving_lab/xdl_parser2/xdl_parser2/test.xml'

class xdl_parser(Node):
    def __init__(self):
        super().__init__('xdl_parser')

        self.done = False

        # client_cb_group = ReentrantCallbackGroup()

        # self.srv = ActionServer(self,Xdl, "convert_xdl",self.xdl_parser_callback,callback_group=client_cb_group)
        # self.BT_client = ActionClient(self,Xdl, 'xdl_action',callback_group=client_cb_group) 

        self.srv = ActionServer(self,Xdl, "convert_xdl",self.xdl_parser_callback)
        self.BT_client = ActionClient(self,Xdl, 'xdl_action') 
        self.get_logger().info("XDL Parser service has been started")
        self.avaiable_tasks_dict = {
            'Add':'<SubTree ID="Add" '
        }

        self.end_tag_dict = {
            'Add':'/AddObjectNode>\n'
        }


    def load_xdl(self):



        script_path = os.path.abspath(__file__)
        path = os.path.dirname(script_path)
        path += '/test.xml'

        root = tk.Tk()
        root.withdraw()

        # path = filedialog.askopenfilename() 
        path = "/home/intelnuc/sdl_ws/src/self_driving_lab/behavior_trees_ros2_example/behavior_tree_ros2_rob8/xdl_xml/blueprint.xml"
        path2 = '/home/lass6230/github/rob8_ws/src/self_driving_lab/behavior_trees_ros2_example/behavior_tree_ros2_rob8/xdl_xml/blueprint.xml'
        try:
            with open(path,"r") as file:
                xml_string = file.read()
                file.close()
        except:
            with open(path2,"r") as file:
                xml_string = file.read()
                file.close()

        return xml_string




    def xdl_parser_callback(self,goal_handle):

        if goal_handle.request.xdl == "" or goal_handle.request.xdl == 'None':
            xdl = self.load_xdl()
        else:
            xdl = goal_handle.request.xdl



        parameters, reagents, procedure, procedureTags = self.parse_xml(xdl)


        behavior_tree, result = self.generate_behavior_tree(procedureTags,procedure)
        
        
        
        if not result:
            result = Xdl.Result()
            result.result = False

        else:
            self.BT_client.wait_for_server()
            request = Xdl.Goal()
            request.xdl = behavior_tree
            # print("DOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNEEEEEEEEEEEEEEE11111111")
            self._send_goal_future = self.BT_client.send_goal_async(request)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            # print("DOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNEEEEEEEEEEEEEEE222222222")
            # rclpy.spin_until_future_complete(self, self._send_goal_future)
            # print("goal actcepted: ",self._send_goal_future.result().accepted)
            
            # self._get_result_future = self._send_goal_future.result().get_result_async()
            # rclpy.spin_until_future_complete(self, self._get_result_future)
            # print("success ",self._get_result_future.result().result.result)
            while(not self.done):
                time.sleep(0.1)
            # print("DOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOONNNNNNNNNNNNNNNNNEEEEEEEEEEEEEEE33333333333333")
            self.done = False
            goal_handle.succeed()
            
            result = Xdl.Result()
            result.result = True
            
            

        return result

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.get_logger().info('Result: {0}'.format(result.result.result))
        self.done = True


        


    def reverse(self,s): 
        str = "" 
        for i in s: 
            str = i + str
        return str

    # Needs a way to check wheter given params can be used with the keywords
    def keyword_to_string(self, keyword, params_dict):

        if keyword in self.avaiable_tasks_dict:
            result = self.avaiable_tasks_dict[keyword]
        else:
            return 'Error'


        for key, value in params_dict.items():
            if value == "" or key == "time":
                pass

            else:
                result += f'{key}="{value}" '
        # result += '/>\n'
        result += ' _autoremap="true"/>\n'
        # result += ' robot_init="{robot_init}"/>\n'




        return result


    def generate_behavior_tree(self,list_of_procedures,params_dict):
        # result = '<?xml version="1.0" encoding="UTF-8"?>\n<root BTCPP_format="4">\n<BehaviorTree ID="Main">\n<Sequence>\n<Script code="robot_init:=0" />\n'
        result = '<?xml version="1.0" encoding="UTF-8"?>\n<root BTCPP_format="4">\n<BehaviorTree ID="Main">\n<Sequence>\n<SubTree ID="init" _autoremap="true"/>\n'

        for index, item in enumerate(list_of_procedures):
            tmp = self.keyword_to_string(item,params_dict[index])


            try:
                assert tmp != 'Error'
            except AssertionError as e:
                self.get_logger().error(f'Error: Invalid or not implemented keyword: {item}')
                success = False
                return f'Error: Invalid or not implemented keyword: {item}', success



            result += tmp


        result += '</Sequence>\n</BehaviorTree>\n</root>'
        success = True


        return result, success


    def parse_xml(self,xml_string):
        try:
            # Parse the XML string
            root = ET.fromstring(xml_string)

            parameters = []
            reagents = []
            procedure = []
            procedureTags = []

            # Iterate over child elements of the root
            for child in root:
                if child.tag == 'Parameters':
                    for parameter in child:
                        parameters.append({
                            'id': parameter.attrib.get('id', ''),
                            'type': parameter.attrib.get('type', ''),
                            'value': parameter.attrib.get('value', '')
                        })
                elif child.tag == 'Reagents':
                    for reagent in child:
                        reagents.append({
                            'id': reagent.attrib.get('id', '')
                        })
                elif child.tag == 'Procedure':
                    for step in child:                    


                        procedureTags.append(step.tag)
                        
                        procedure.append({
                            'reagent': step.attrib.get('reagent', ''),
                            'vessel': step.attrib.get('vessel', ''),
                            'amount': step.attrib.get('amount', ''),
                            'time': step.attrib.get('time', '')
                        })

            return parameters, reagents, procedure, procedureTags

        except ET.ParseError as e:
            print("Error parsing the XML:", e)
            return [], [], []

# Example usage:
def main(args=None):
    rclpy.init()
    parser = xdl_parser()

    executor = MultiThreadedExecutor()
    executor.add_node(parser)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()