# ROS2 imports
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time', default="false")

    aruco_params_file = os.path.join(
        get_package_share_directory('camera'),
        'config',
        'aruco_parameters.yaml'
    )

    with open(aruco_params_file, 'r') as file:
        config = yaml.safe_load(file)

    config = config["/aruco_node"]["ros__parameters"]

    # declare configuration parameters
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='false',
        description='launch rviz',
    )

    marker_size_arg = DeclareLaunchArgument(
        name='marker_size',
        default_value=str(config['marker_size']),
        description='Size of the aruco marker in meters',
    )

    debug_arg = DeclareLaunchArgument(
        name='debug',
        default_value=str(config['debug']),
        description='Size of the aruco marker in meters',
    )

    aruco_dictionary_id_arg = DeclareLaunchArgument(
        name='aruco_dictionary_id',
        default_value=config['aruco_dictionary_id'],
        description='ID of the aruco dictionary to use',
    )

    image_topic_arg = DeclareLaunchArgument(
        name='image_topic',
        default_value=config['image_topic'],
        description='Name of the image RGB topic to subscribe to',
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        name='camera_info_topic',
        default_value=config['camera_info_topic'],
        description='Name of the camera info topic to subscribe to',
    )

    camera_frame_arg = DeclareLaunchArgument(
        name='camera_frame',
        default_value=config['camera_frame'],
        description='Name of the camera frame where the estimated pose will be',
    )

    detected_markers_topic_arg = DeclareLaunchArgument(
        name='detected_markers_topic',
        default_value=config['detected_markers_topic'],
        description='Name of the topic to publish the detected markers messages',
    )

    markers_visualization_topic_arg = DeclareLaunchArgument(
        name='markers_visualization_topic',
        default_value=config['markers_visualization_topic'],
        description='Name of the topic to publish the pose array for visualization of the markers',
    )

    

    aruco_node = Node(
        package='camera',
        executable='aruco_pose_estimator',
        parameters=[{
            "marker_size": LaunchConfiguration('marker_size'),
            "debug": LaunchConfiguration('debug'),
            "aruco_dictionary_id": LaunchConfiguration('aruco_dictionary_id'),
            "image_topic": LaunchConfiguration('image_topic'),
            "camera_info_topic": LaunchConfiguration('camera_info_topic'),
            "camera_frame": LaunchConfiguration('camera_frame'),
            "detected_markers_topic": LaunchConfiguration('detected_markers_topic'),
            "markers_visualization_topic": LaunchConfiguration('markers_visualization_topic'),
        }],
        output='screen',
        emulate_tty=True
    )


    cam_feed_launch_file = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    camera_feed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cam_feed_launch_file),
        launch_arguments={
            "pointcloud.enable": "true",
            "enable_color": "true",
            "rgb_camera.profile": "1920,1080,6",
            # "align_depth.enable": "true", #jeg dræber din mor hvis du kommenterer denne ind
            "clip_distance": "1.0",
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    rviz_config_file = os.path.join(get_package_share_directory('camera'), 'config', 'cam_default.rviz')
    print(rviz_config_file)
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        use_rviz_arg,

        marker_size_arg,
        debug_arg,
        aruco_dictionary_id_arg,
        image_topic_arg,
        camera_info_topic_arg,
        camera_frame_arg,
        detected_markers_topic_arg,
        markers_visualization_topic_arg,

        # Nodes
        aruco_node, 
        camera_feed_node,
        rviz2_node,
        
    ])
