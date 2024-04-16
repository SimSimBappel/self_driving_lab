import xml.etree.ElementTree as ET
from xdl_parser_interface.srv import XdlConversion
from behavior_tree_ros2_actions.srv import Xdl
from rclpy.node import Node
import tkinter as tk
from tkinter import filedialog
import rclpy
import os

sussy = '/home/lilholt/github/sdl_ws/src/self_driving_lab/xdl_parser2/xdl_parser2/test.xml'

class xdl_parser(Node):
    def __init__(self):
        super().__init__('xdl_parser')
        self.srv = self.create_service(Xdl, "convert_xdl",self.xdl_parser_callback)
        self.BT_client = self.create_client(Xdl, 'xdl_service') 
        self.get_logger().info("XDL Parser service has been started")
        self.avaiable_tasks_dict = {
            'Add':'<AddObjectNode '
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

        path = filedialog.askopenfilename() 
        with open(path,"r") as file:
            xml_string = file.read()
            file.close()

        return xml_string




    def xdl_parser_callback(self,request,response):

        if request.xdl == "" or request.xdl == 'None':
            xdl = self.load_xdl()
        else:
            xdl = request.xdl


        parameters, reagents, procedure, procedureTags = self.parse_xml(xdl)


        behavior_tree, result = self.generate_behavior_tree(procedureTags,procedure)
        if not result:
            response.result = False

        else:
            response.result = True
            self.BT_client.wait_for_service()
            request = Xdl.Request()
            request.xdl = behavior_tree
            future = self.BT_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

        return response



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

        result += '/>\n'



        return result


    def generate_behavior_tree(self,list_of_procedures,params_dict):
        result = '<?xml version="1.0" encoding="UTF-8"?>\n<root BTCPP_format="4">\n<BehaviorTree ID="Main">\n<Sequence>\n'

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
        print(result)


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


    try:
        rclpy.spin(parser)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()