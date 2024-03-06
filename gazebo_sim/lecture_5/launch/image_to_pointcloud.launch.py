import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[
                        ('/image_rect', '/aligned_depth_to_color/image_raw'),
                        ('/camera_info', '/aligned_depth_to_color/image_raw')
                    ]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])