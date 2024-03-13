from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Define nodes or actions in the main launch file
        
        
        # Include the other launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("camera"),'launch','rs.launch.py'))
        ),
        Node(
            package='camera',
            executable='aruco_detection_action',
            name='aruco_detection_action_server'
        ),
    ])
