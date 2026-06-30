import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    alignment_args = LaunchConfiguration('marker_map')
    mapping_config_folder = os.path.join(get_package_share_directory("gorm_mapping"), "config")
    map_alignment_path = os.path.join(mapping_config_folder, "map_alignment.yaml")


    aligner_node = Node(
        package='gorm_mapping',
        executable='world_map_aligner',
        name='world_map_aligner',
        output='screen',
        parameters=[map_alignment_path],
        emulate_tty=True,
    )


    aruco_tf_front_node = Node(
            package='gorm_mapping',
            executable='aruco_tf_node',
            name='aruco_tf_front_node',
            output='screen',
            parameters=[{
                'image_topic': '/zed_front/zed_front/rgb/image_rect_color/compressed',
                'camera_info_topic': '/zed_front/zed_front/rgb/camera_info',
                'marker_length': 0.15,
                'dictionary': 'DICT_5X5_100',
                'tf_prefix': '',
                'child_frame_prefix': 'aruco_marker_',
                'publish_debug_image': False,
                'debug_image_topic': '/aruco/debug_image_front',
                'use_image_header_stamp': True,
            }]
        )
    
    aruco_tf_back_node = Node(
            package='gorm_mapping',
            executable='aruco_tf_node',
            name='aruco_tf_back_node',
            output='screen',
            parameters=[{
                'image_topic': '/zed_back/zed_back/rgb/image_rect_color/compressed',
                'camera_info_topic': '/zed_back/zed_back/rgb/camera_info',
                'marker_length': 0.15,
                'dictionary': 'DICT_5X5_100',
                'tf_prefix': '',
                'child_frame_prefix': 'aruco_marker_',
                'publish_debug_image': False,
                'debug_image_topic': '/aruco/debug_image_back',
                'use_image_header_stamp': True,
            }]
        )


    return LaunchDescription([

        DeclareLaunchArgument('marker_map', default_value='map_alignment.yaml', description='What marker map to use for alignment'),

        LogInfo(msg='Launching Aruco-based Map Aligner'),
        aligner_node,
        aruco_tf_front_node,
        aruco_tf_back_node
    ])