import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the path to the moveit_planners_ompl package
    moveit_planners_ompl_share_dir = get_package_share_directory('moveit_planners_ompl')

    # Find the path to the mtc_pour package
    mtc_pour_share_dir = get_package_share_directory('mtc_pour')

    # Define launch arguments
    planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='arm',
        description='Name of the planning group'
    )

    constraints_file_arg = DeclareLaunchArgument(
        'constraints_file',
        default_value=f'{mtc_pour_share_dir}/config/upright_constraint.yaml',
        description='Path to the constraints file'
    )

    # Create the launch description
    return LaunchDescription([
        planning_group_arg,
        constraints_file_arg,
        Node(
            package='moveit_planners_ompl',
            executable='ompl_interface_node',
            name='ompl_interface_node',
            parameters=[{
                'planning_group': launch.substitutions.LaunchConfiguration('planning_group'),
                'constraints_yaml': launch.substitutions.LaunchConfiguration('constraints_file')
            }],
            output='screen'
        )
    ])