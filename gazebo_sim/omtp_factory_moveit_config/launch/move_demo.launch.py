import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    
    moveit_config = (MoveItConfigsBuilder("omtp_factory", package_name="omtp_factory_moveit_config")
                     .parameter("use_sim_time",True)
                     .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
                     .to_moveit_configs()
    )
    move_group_demo = Node(
        name="move_demo",
        package="omtp_factory_moveit_config",
        executable="move_demo",
        output="screen",
        parameters=[
           # moveit_config.robot_description,
            #moveit_config.robot_description_semantic,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription(
        [
            move_group_demo,
            

            #tutorial_arg,
            #db_arg,
            #rviz_node,
            #rviz_node_tutorial,
            #static_tf,
            #robot_state_publisher,
            #run_move_group_node,
            #ros2_control_node,
            #mongodb_server_node,
        ]
        #+ load_controllers
    )
