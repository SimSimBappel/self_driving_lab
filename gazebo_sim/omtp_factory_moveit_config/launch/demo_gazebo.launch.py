from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils.launches import *
from launch.event_handlers import OnExecutionComplete, OnProcessExit
def generate_launch_description():

    


    moveit_config = (MoveItConfigsBuilder("omtp_factory", package_name="omtp_factory_moveit_config")
                     .parameter("use_sim_time",True)
                     .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True)
                    # .sensors_3d(os.path.join(get_package_share_directory("omtp_factory_moveit_config"),'config','sensors_depthmap.yaml'))
                     .to_moveit_configs()
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    static_tf_oak = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "1.2", "2.0", "0.0", "0.0", "0.0", "world", "oak-d-base-frame"],
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
                    # launch_arguments={'/home/lass6230/github/OMTP_excersizes/lecture_1/src/external_packages/ARIAC_logic_cameraxtra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )
    # robot_description_path = os.path.join(
    #                         get_package_share_directory("omtp_lecture1"),
    #                         'urdf/omtp_custom.urdf.xacro')
    # print(robot_description_path)
    # robot_description_content = open(robot_description_path).read()
    
    # robot_description = {"robot_description": robot_description_content}
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', {'robot_description': moveit_config.robot_description},
                                   '-entity', 'omtp_factory'],
                        output='screen')

    # tem = generate_demo_launch(moveit_config)

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


    # delayed_moveit_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=tem,
    #         on_exit=[spawn_entity],
    #     )
    # )
    # pkg_share = FindPackageShare(package='ariac_gazebo').find('ariac_gazebo')
  
    # # sensor_config = LaunchConfiguration("sensor_config").perform(context)
    # user_config_path = os.path.join(pkg_share, 'config', 'sensor' + ".yaml")
    # sensor_tf_broadcaster = Node(
    #     package='ariac_gazebo',
    #     executable='sensor_tf_broadcaster.py',
    #     output='screen',
    #     arguments=[user_config_path],
    #     parameters=[
    #         {"use_sim_time": True},
    #     ]
    # )
    
    # trial_name = LaunchConfiguration("trial_name").perform(context)
    # trial_config_path = os.path.join(pkg_share, 'config', 'trials', 'kitting' + ".yaml")
    # Objects TF

    # object_tf_broadcaster = Node(
    #     package='ariac_gazebo',
    #     executable='object_tf_broadcaster.py',
    #     output='screen',
    #     parameters=[
    #         {"use_sim_time": True},
    #         {'trial_config_path': trial_config_path},
    #     ]
    # )



    


    return LaunchDescription(
        [   
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
            static_tf_oak,
            # sensor_tf_broadcaster,
            # object_tf_broadcaster,

            
            DeclareLaunchArgument("trial_name", default_value="kitting", description="name of trial"),
            DeclareLaunchArgument("sensor_config", default_value="sensors", description="name of user configuration file"),
 
        ]
    )
    
