from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils.launches import *
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import LaunchConfiguration #, Command, FindExecutable, PathJoinSubstitution
# import xacro
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction #, ExecuteProcess, RegisterEventHandler
# from launch.event_handlers import OnExecutionComplete, OnProcessExit

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulated time'
    )


    moveit_config = (MoveItConfigsBuilder("omtp_factory", package_name="omtp_factory_moveit_config")
                     .parameter("use_sim_time",True)
                     .planning_pipelines(
            # pipelines=["ompl", "pilz_industrial_motion_planner","chomp"],
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
                     .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True)
                     .to_moveit_configs()
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "panda1_link0", "base_link"],
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),{'use_sim_time': True}],
    )
    
    package_name='omtp_lecture1'
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', {'robot_description': moveit_config.robot_description},
                                   '-entity', 'omtp_factory'],
                        output='screen')


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,{'use_sim_time': True}],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{'use_sim_time': True}],
    )
    ros2_controllers_path = os.path.join(
        get_package_share_directory("omtp_factory_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path,{'use_sim_time': True}],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    panda_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_1_controller",
            "panda_1_gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    
    included_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("camera"), 'launch', 'pose_estimation.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )


    return LaunchDescription(
        [   
            use_sim_time_arg,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            gazebo,
            spawn_entity,
            # delayed_moveit_drive_spawner
            rviz_node,
            panda_controller_spawner,
            ros2_control_node,
            run_move_group_node,
            static_tf,

            
            DeclareLaunchArgument("trial_name", default_value="kitting", description="name of trial"),
            DeclareLaunchArgument("sensor_config", default_value="sensors", description="name of user configuration file"),

            # launch image pipeline
            included_launch_description,
            
        ]
    )
    
