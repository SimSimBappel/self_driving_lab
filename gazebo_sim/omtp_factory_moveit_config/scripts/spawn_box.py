#!/usr/bin/env python3
import random
import rclpy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetWorldProperties

"""
This script demonstrates how to spawn a simple box model in Gazebo using the SpawnModel service.
The box is spawned at a fixed position and orientation in the world frame.
The box is given a random unique name using a random number.
The box is given a green color using the ambient material property.

To run this script, first start Gazebo and then run this script in a separate terminal.

Author: Simon Bøgh
Date: 2024
"""

def spawn_box():
    """ Spawns a simple box model in Gazebo using the SpawnModel service. """
    rclpy.init_node('spawn_box')

    rclpy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rclpy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        # Give the box a random unique name using a random number
        model_name = 'box' + str(random.randint(0, 100000))
        model_xml = '''
        <sdf version='1.6'>
            <model name='my_box'>
                <pose>0 0 1 0 0 0</pose>
                <link name='link'>
                    <inertial>
                    <mass>0.1</mass>
                    <inertia>
                        <ixx>0.00004166667</ixx>
                        <iyy>0.00004166667</iyy>
                        <izz>0.00004166667</izz>
                    </inertia>
                    </inertial>
                    <collision name='collision'>
                        <geometry>
                            <box>
                                <size>0.05 0.05 0.05</size>
                            </box>
                        </geometry>
                        <max_contacts>10</max_contacts>
                        <surface>
                            <contact>
                            <ode>
                                <max_vel>5.0</max_vel>
                                <min_depth>0.001</min_depth> 
                                <kp>10000</kp>               
                                <kd>0</kd>                   
                            </ode>
                            </contact>

                            <bounce>
                            <restitution_coefficient>0.5</restitution_coefficient>  <!-- "max_vel" must be higher than 0.0 -->
                            <threshold>0.01</threshold>
                            </bounce>

                            <friction>
                            <ode>
                                <mu>0.95</mu>
                                <mu2>0.95</mu2>
                            </ode>
                            </friction>
                        </surface>
                    </collision>
                    <visual name='visual'>
                        <geometry>
                            <box>
                                <size>0.05 0.05 0.05</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>0 1 0 1</ambient> <!-- Green color -->
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>
        '''

        # Set the pose of the box
        model_pose = Pose()
        model_pose.position.x = 1
        model_pose.position.y = 0
        model_pose.position.z = 1
        model_pose.orientation.x = 0.0
        model_pose.orientation.y = 0.0
        model_pose.orientation.z = 0.0
        
        # Call the service to spawn the box
        spawn_model(model_name, model_xml, '', model_pose, 'world')
        rclpy.loginfo("Model spawned.")
    except rclpy.ServiceException as e:
        rclpy.logerr("Service call failed: %s" % e)


def get_box_names():
    """ Returns the names of all box models in the world. """
    rclpy.init_node('get_box_names')
    rclpy.wait_for_service('/gazebo/get_world_properties')
    try:
        # Call the service to get the names of all models in the world
        get_world_properties = rclpy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world_properties = get_world_properties()
        # Filter out the names of the box models
        box_names = [name for name in world_properties.model_names if name.startswith('box')]
        print("Box names in the world:", box_names)
        return box_names
    except rclpy.ServiceException as e:
        print("Service call failed: %s" % e)
        return []


def remove_all_boxes(box_names):
    """ Removes all box models from the world. """
    rclpy.wait_for_service('/gazebo/delete_model')
    delete_model = rclpy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    
    for model_name in box_names:
        try:
            resp = delete_model(model_name)
            rclpy.loginfo(f"Model {model_name} removed: {resp.success}")
        except rclpy.ServiceException as e:
            rclpy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':
    try:
        # Ask the user whether to spawn a box or remove all boxes by pressing 1 or 2
        user_input = input("1: Spawn a box\n2: Remove all boxes: ")
        if user_input == '1':
            # Ask how many boxes to spawn
            user_input = input("How many boxes to spawn? ")
            for i in range(int(user_input)):
                spawn_box()
        elif user_input == '2':
            # Remove all boxes
            boxes = get_box_names()
            remove_all_boxes(boxes)
        else:
            print("Invalid input.")
    
    except rclpy.ROSInterruptException:
        pass
    