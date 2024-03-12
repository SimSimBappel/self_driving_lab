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
                     .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner","chomp"],
            default_planning_pipeline="ompl",
        )
                     .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True)
                    .sensors_3d(os.path.join(get_package_share_directory("omtp_factory_moveit_config"),'config','sensors_depthmap.yaml'))
                     .to_moveit_configs()
    )
    # planning_pipelines_config = {
    #     "default_planning_pipeline": "ompl",
    #     "planning_pipelines": ["ompl", "pilz_industrial_motion_planner"],
    #     "pilz_industrial_motion_planner": {
    #         "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
    #         "request_adapters": "",
    #         "start_state_max_bounds_error": 0.1,
    #     },
    #     "ompl": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 0.1,
    #     },
    # }
    # ompl_planning_yaml = load_yaml(
    #     "omtp_factory_moveit_config", "config/ompl_planning.yaml"
    # )
    # pilz_planning_yaml = load_yaml(
    #     "omtp_factory_moveit_config", "config/pilz_industrial_motion_planner_planning.yaml"
    # )
    # planning_pipelines_config["ompl"].update(ompl_planning_yaml)
    # planning_pipelines_config["pilz_industrial_motion_planner"].update(pilz_planning_yaml)

    # move_group_capabilities = {
    #     "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    # }

    move_group_demo = Node(
        name="move_robot_server",
        package="manipulator",
        executable="move_robot_server",
        output="screen",
        parameters=[
           # moveit_config.robot_description,
            #moveit_config.robot_description_semantic,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            # moveit_config.to_dict(),
            # move_group_capabilities,
            # planning_pipelines_config,
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription(
        [
            move_group_demo,
            # sim_mode_arg,
            # com_arg,

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






# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
#                             Shutdown)
# from launch.conditions import UnlessCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# import yaml


# def load_yaml(package_name, file_path):
#     package_path = get_package_share_directory(package_name)
#     absolute_file_path = os.path.join(package_path, file_path)

#     try:
#         with open(absolute_file_path, 'r') as file:
#             return yaml.safe_load(file)
#     except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
#         return None


# def generate_launch_description():
#     robot_ip_parameter_name = 'robot_ip'
#     use_fake_hardware_parameter_name = 'use_fake_hardware'
#     fake_sensor_commands_parameter_name = 'fake_sensor_commands'

#     robot_ip = LaunchConfiguration(robot_ip_parameter_name)
#     use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
#     fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)

#     # Command-line arguments

#     db_arg = DeclareLaunchArgument(
#         'db', default_value='False', description='Database flag'
#     )

#     # planning_context
#     franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
#                                      'panda_arm.urdf.xacro')
#     robot_description_config = Command(
#         [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',
#          ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
#          ' fake_sensor_commands:=', fake_sensor_commands])

#     robot_description = {'robot_description': robot_description_config}

#     franka_semantic_xacro_file = os.path.join(get_package_share_directory('franka_moveit_config'),
#                                               'srdf',
#                                               'panda_arm.srdf.xacro')
#     robot_description_semantic_config = Command(
#         [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=true']
#     )
#     robot_description_semantic = {
#         'robot_description_semantic': robot_description_semantic_config
#     }

#     kinematics_yaml = load_yaml(
#         'franka_moveit_config', 'config/kinematics.yaml'
#     )

#     # Planning Functionality
#     ompl_planning_pipeline_config = {
#         'move_group': {
#             'planning_plugin': 'ompl_interface/OMPLPlanner',
#             'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
#                                 'default_planner_request_adapters/ResolveConstraintFrames '
#                                 'default_planner_request_adapters/FixWorkspaceBounds '
#                                 'default_planner_request_adapters/FixStartStateBounds '
#                                 'default_planner_request_adapters/FixStartStateCollision '
#                                 'default_planner_request_adapters/FixStartStatePathConstraints',
#             'start_state_max_bounds_error': 0.1,
#         }
#     }
#     ompl_planning_yaml = load_yaml(
#         'franka_moveit_config', 'config/ompl_planning.yaml'
#     )
#     ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

#     # Trajectory Execution Functionality
#     moveit_simple_controllers_yaml = load_yaml(
#         'franka_moveit_config', 'config/panda_controllers.yaml'
#     )
#     moveit_controllers = {
#         'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
#         'moveit_controller_manager': 'moveit_simple_controller_manager'
#                                      '/MoveItSimpleControllerManager',
#     }

#     trajectory_execution = {
#         'moveit_manage_controllers': True,
#         'trajectory_execution.allowed_execution_duration_scaling': 1.2,
#         'trajectory_execution.allowed_goal_duration_margin': 0.5,
#         'trajectory_execution.allowed_start_tolerance': 0.01,
#     }

#     planning_scene_monitor_parameters = {
#         'publish_planning_scene': True,
#         'publish_geometry_updates': True,
#         'publish_state_updates': True,
#         'publish_transforms_updates': True,
#     }

#     # Start the actual move_group node/action server
#     run_move_group_node = Node(
#         package='moveit_ros_move_group',
#         executable='move_group',
#         output='screen',
#         parameters=[
#             robot_description,
#             robot_description_semantic,
#             kinematics_yaml,
#             ompl_planning_pipeline_config,
#             trajectory_execution,
#             moveit_controllers,
#             planning_scene_monitor_parameters,
#         ],
#     )

#     # RViz
#     rviz_base = os.path.join(get_package_share_directory('franka_moveit_config'), 'rviz')
#     rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='log',
#         arguments=['-d', rviz_full_config],
#         parameters=[
#             robot_description,
#             robot_description_semantic,
#             ompl_planning_pipeline_config,
#             kinematics_yaml,
#         ],
#     )

#     # Publish TF
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='both',
#         parameters=[robot_description],
#     )

#     ros2_controllers_path = os.path.join(
#         get_package_share_directory('franka_moveit_config'),
#         'config',
#         'panda_ros_controllers.yaml',
#     )
#     ros2_control_node = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[robot_description, ros2_controllers_path],
#         remappings=[('joint_states', 'franka/joint_states')],
#         output={
#             'stdout': 'screen',
#             'stderr': 'screen',
#         },
#         on_exit=Shutdown(),
#     )

#     # Load controllers
#     load_controllers = []
#     for controller in ['panda_arm_controller', 'joint_state_broadcaster']:
#         load_controllers += [
#             ExecuteProcess(
#                 cmd=['ros2 run controller_manager spawner {}'.format(controller)],
#                 shell=True,
#                 output='screen',
#             )
#         ]

#     joint_state_publisher = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         name='joint_state_publisher',
#         parameters=[
#             {'source_list': ['franka/joint_states', 'panda_gripper/joint_states'], 'rate': 30}],
#     )

#     franka_robot_state_broadcaster = Node(
#             package='controller_manager',
#             executable='spawner',
#             arguments=['franka_robot_state_broadcaster'],
#             output='screen',
#             condition=UnlessCondition(use_fake_hardware),
#     )

#     move_group_demo = Node(
#         name="move_robot_server",
#         package="ar4_moveit_config",
#         executable="move_robot_server",
#         output="screen",
#         parameters=[
#            # moveit_config.robot_description,
#             #moveit_config.robot_description_semantic,
#             robot_description,
#             robot_description_semantic,
#             kinematics_yaml,
#             ompl_planning_pipeline_config
#         ],
#     )

#     # robot_arg = DeclareLaunchArgument(
#     #     robot_ip_parameter_name,
#     #     description='Hostname or IP address of the robot.')

#     # use_fake_hardware_arg = DeclareLaunchArgument(
#     #     use_fake_hardware_parameter_name,
#     #     default_value='false',
#     #     description='Use fake hardware')
#     # fake_sensor_commands_arg = DeclareLaunchArgument(
#     #     fake_sensor_commands_parameter_name,
#     #     default_value='false',
#     #     description="Fake sensor commands. Only valid when '{}' is true".format(
#     #         use_fake_hardware_parameter_name))
#     # gripper_launch_file = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource([PathJoinSubstitution(
#     #         [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
#     #     launch_arguments={'robot_ip': robot_ip,
#     #                       use_fake_hardware_parameter_name: use_fake_hardware}.items(),
#     )
#     return LaunchDescription(
#         [ #robot_arg,
#         #  use_fake_hardware_arg,
#         #  fake_sensor_commands_arg,
#         #  db_arg,
#         #  rviz_node,
#         #  robot_state_publisher,
#         #  run_move_group_node,
#         #  ros2_control_node,
#         #  joint_state_publisher,
#         #  franka_robot_state_broadcaster,
#         #  gripper_launch_file
#             move_group_demo
            
#          ]
#         # + load_controllers
#     )