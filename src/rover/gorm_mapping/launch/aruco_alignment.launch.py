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
    mapping_config_folder = os.path.join(get_package_share_directory("gorm_mapping"), "config")
    map_alignment_path = os.path.join(mapping_config_folder, "map_alignment.yaml")


    aligner_node = Node(
        package='gorm_mapping',
        executable='world_map_aligner',
        name='world_map_aligner',
        output='screen',
        # If you use a YAML file:
        #['/home/jonas/rover-software/src/rover/gorm_mapping/config/map_alignment.yaml'],
        parameters=[map_alignment_path],
        # parameters=[{   # [map_alignment_path] if map_alignment_path.perform(None) != '' else 
        #     # Inline parameters (works without YAML; adjust as needed)
        #     'world_frame': 'world',
        #     'map_frame': 'map',
        #     'marker_frame_prefix': 'aruco_marker_',
        #     'min_markers': 2,
        #     'recompute': True,
        #     'timer_period': 1.0,
        #     'known_markers_json': '{"51":{"x":0.5,"y":0.0},"52":{"x":0.5,"y":3.0},"53":{"x":-1.0,"y":3.0},"54":{"x":-0.5,"y":6.0},"55":{"x":0.5,"y":9.0},"56":{"x":0.0,"y":10.0}}',
        #     # 'known_markers_json': '{"1":{"x":2.0,"y":0.5},"2":{"x":5.0,"y":0.5},"7":{"x":2.0,"y":3.0}}'
        #     # 'known_markers': {
        #     #     '1': {'x': 2.0, 'y': 0.5},
        #     #     '2': {'x': 5.0, 'y': 0.5},
        #     #     '7': {'x': 2.0, 'y': 3.0},
        #     # },
        #     'smooth_window': 10,
        # }],
        emulate_tty=True,
    )


    aruco_tf_node = Node(
            package='gorm_mapping',
            executable='aruco_tf_node',
            name='aruco_tf_node',
            output='screen',
            parameters=[{
                'image_topic': '/zed_front/zed/rgb/image_rect_color/compressed',
                'camera_info_topic': '/zed_front/zed/rgb/camera_info',
                'marker_length': 0.15,
                'dictionary': 'DICT_5X5_100',
                'tf_prefix': '',
                'child_frame_prefix': 'aruco_marker_',
                'publish_debug_image': False,
                'debug_image_topic': '/aruco/debug_image',
                'use_image_header_stamp': True,
            }]
        )


    return LaunchDescription([
        LogInfo(msg='Launching Aruco-based Map Aligner'),
        aligner_node,
        aruco_tf_node
    ])