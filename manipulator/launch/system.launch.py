# this lauch file is used to launch the manipulator; move_server, franka_control and camera 


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
#                             Shutdown)
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Arguments
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    load_gripper_parameter_name = 'load_gripper'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name, default=True)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)


    # Command-line arguments

    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulated time'
    )





    # planning_context
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'panda_arm.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper,
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])

    robot_description = {'robot_description': robot_description_config}

    franka_semantic_xacro_file = os.path.join(get_package_share_directory('franka_moveit_config'),
                                              'srdf',
                                              'panda_arm.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=', load_gripper]
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'franka_moveit_config', 'config/kinematics.yaml'
    )

    camera_sensor = load_yaml(
        'franka_moveit_config', 'config/sensors_depthmap.yaml'
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl", "pilz_industrial_motion_planner"],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        },
        "pilz_industrial_motion_planner": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": "",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(
        'franka_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    

    pilz_planning_yaml = load_yaml(
        "franka_moveit_config", "config/pilz_industrial_motion_planner_planning.yaml"
    )
    joint_limit_yaml = load_yaml(
        "franka_moveit_config", "config/pilz_industrial_motion_planner_planning.yaml"
    )
    pilz_cartesian_yaml = load_yaml(
        "franka_moveit_config", "config/pilz_industrial_motion_planner_planning.yaml"
    )

    joint_limits_yaml = load_yaml(
        "franka_moveit_config", "config/joint_limits.yaml"
    )
    pilz_cartesian_limits_yaml = load_yaml(
        "franka_moveit_config", "config/pilz_cartesian_limits.yaml"
    )
    robot_description_planning = {
        "robot_description_planning": {
            **joint_limits_yaml,
            **pilz_cartesian_limits_yaml,
        }
    }

    ompl_planning_pipeline_config["pilz_industrial_motion_planner"].update(pilz_planning_yaml)
    

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'franka_moveit_config', 'config/panda_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # camera_sensor = load_yaml(
    #     'franka_moveit_config', 'config/sensors_depthmap.yaml'
    # )

    octomap_config = {'octomap_frame': 'camera_link', 
                  'octomap_resolution': 0.02,
                  'max_range': 1.0,
                  'min_range': 0.2}

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            robot_description_planning,
            camera_sensor,
            octomap_config,
        ],
    )

    move_server = Node(
        name="move_robot_server",
        package="manipulator",
        executable="move_robot_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            octomap_config,
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory('manipulator'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'system_default2.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            camera_sensor,
            octomap_config,
        ],
    )

    robot_lookup_t = Node(
        package='camera',
        executable='lookup',
        name='lookup',
        output='both',
        parameters=[robot_description],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'config',
        'panda_ros_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    for controller in ['panda_arm_controller', 'joint_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    # Warehouse mongodb server
    db_config = LaunchConfiguration('db')
    mongodb_server_node = Node(
        package='warehouse_ros_mongo',
        executable='mongo_wrapper_ros.py',
        parameters=[
            {'warehouse_port': 33829},
            {'warehouse_host': 'localhost'},
            {'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection'},
        ],
        output='screen',
        condition=IfCondition(db_config)
    )

    pgsql_node = Node(
        package='pgsql_services',
        executable='database_service',
        output='screen'
    )


    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['franka/joint_states', 'panda_gripper/joint_states'], 'rate': 30}],
    )
    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        default_value='192.168.2.30',
        description='Hostname or IP address of the robot.')

    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description='Use fake hardware')
    
    load_gripper_arg = DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.')
    
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))
    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={'robot_ip': robot_ip,
                          use_fake_hardware_parameter_name: use_fake_hardware}.items(),
        condition=IfCondition(load_gripper)
    )

    launch_pose_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("camera"), 'launch', 'pose_estimation.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # launch_moveit = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("franka_moveit_config"), 'launch', 'moveit.launch.py')),
    #     launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    # )

    return LaunchDescription(
        [robot_arg,
         use_sim_time_arg,
         use_fake_hardware_arg,
         fake_sensor_commands_arg,
         load_gripper_arg,
         db_arg,
         rviz_node,
         robot_state_publisher,
         run_move_group_node,
         ros2_control_node,
         mongodb_server_node,
         joint_state_publisher,
         gripper_launch_file,
         move_server,
         robot_lookup_t,
         pgsql_node,
        #  launch_moveit,
         launch_pose_estimation,
         ]
        + load_controllers
    )
