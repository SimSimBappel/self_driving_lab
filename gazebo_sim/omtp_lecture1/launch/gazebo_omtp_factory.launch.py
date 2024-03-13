import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
                            'urdf/omtp_custom_factory.urdf.xacro')
    print(robot_description_path)
    robot_description_content = open(robot_description_path).read()
    
    robot_description = {"robot_description": robot_description_content}

    # pkg_name = 'omtp_lecture1'
    # file_subpath = 'urdf/omtp_custom.urdf.xacro'


    # # Use xacro to process the file
    # xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    # robot_description_raw = xacro.process_file(xacro_file).toxml()

    # use_sim_time = {"use_sim_time": True}
    rviz_config_file = PathJoinSubstitution(
         [FindPackageShare("omtp_lecture1"), "rviz", "omtp.rviz"]
    )

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     parameters=[robot_description,{'use_sim_time': True}],
    # )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[robot_description,{'use_sim_time': True}],
    )

    # start_joint_state_publisher_cmd = Node(
    #     condition=UnlessCondition(gui),
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher'
    #     )

    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description_raw,
    #     'use_sim_time': True}] # add other parameters here if required
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,{'use_sim_time': True}],
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

    package_name='omtp_lecture1'
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', robot_description,
                                   '-entity', 'omtp_factory'],
                        output='screen')

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        # rviz_node,
        gazebo,
        spawn_entity,
    ]



    return LaunchDescription(declared_arguments + nodes)