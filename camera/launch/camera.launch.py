from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Get the value of use_sim_time argument
    use_sim_time = LaunchConfiguration('use_sim_time', default="false")

    # Create a list to hold launch description actions
    ld_actions = []

    # Include the other launch file only if use_sim_time is false
    ld_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("camera"), 'launch', 'rs.launch.py')
            ),
            condition=UnlessCondition(use_sim_time)
        )
    )

    # Add the node for aruco_detection_action_server
    ld_actions.append(
        Node(
            package='camera',
            executable='aruco_detection_action',
            name='aruco_detection_action_server'
        )
    )

    # Return the launch description
    print(ld_actions)
    return LaunchDescription(ld_actions)
