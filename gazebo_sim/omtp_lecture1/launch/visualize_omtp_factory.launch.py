import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []


    # get urdf path

    #robot_description_path = PathJoinSubstitution(
    #            [FindPackageShare(description_package), "urdf", description_file]
    # #        )
    # robot_description_path = os.path.join(
    #                         get_package_share_directory("omtp_lecture1"),
    #                         'urdf/omtp.urdf.xacro')
    robot_description_path = os.path.join(
                            get_package_share_directory("omtp_lecture1"),
                            'urdf/omtp_custom.urdf.xacro')
    print(robot_description_path)
    robot_description_content = open(robot_description_path).read()
    
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
         [FindPackageShare("omtp_lecture1"), "rviz", "omtp.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # start_joint_state_publisher_cmd = Node(
    #     condition=UnlessCondition(gui),
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher'
    #     )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            
        ],
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)