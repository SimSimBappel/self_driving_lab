import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import FindExecutable, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the path to the tams_ur5_bringup package
    tams_ur5_bringup_share_dir = get_package_share_directory('tams_ur5_bringup')

    # Find the path to the tams_ur5_description package
    tams_ur5_description_share_dir = get_package_share_directory('tams_ur5_description')

    # Find the path to the tams_ur5_setup_description package
    tams_ur5_setup_description_share_dir = get_package_share_directory('tams_ur5_setup_description')

    # Find the path to the tams_ur5_setup_moveit_config package
    tams_ur5_setup_moveit_share_dir = get_package_share_directory('tams_ur5_setup_moveit_config')

    # Find the path to the mtc_pour package
    mtc_pour_share_dir = get_package_share_directory('mtc_pour')

    # Define launch description
    return LaunchDescription([
        # Include tams_ur5_drivers.launch from tams_ur5_bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{tams_ur5_bringup_share_dir}/launch/tams_ur5_drivers.launch'
            ),
            launch_arguments={
                'gripper_mode': 'wide_pinch'
            }.items(),
        ),

        # Include tams_ur5_setup_upload.launch from tams_ur5_setup_description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{tams_ur5_setup_description_share_dir}/launch/tams_ur5_setup_upload.launch'
            ),
            launch_arguments={
                'joint_ranges_config': f'{tams_ur5_description_share_dir}/config/joint_ranges/elbow_up.yaml'
            }.items(),
        ),

        # Include move_group.launch from tams_ur5_setup_moveit_config
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{tams_ur5_setup_moveit_share_dir}/launch/move_group.launch'
            )
        ),

        # Set parameter for move_group constraint_approximations_path
        Node(
            package='rosparam',
            executable='rosparam',
            arguments=['set', 'move_group/constraint_approximations_path', f'{mtc_pour_share_dir}/cadb']
        )
    ])

