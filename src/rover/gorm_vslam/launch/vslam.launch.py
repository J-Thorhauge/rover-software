import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    
    # static_transform_publisher_fcam = Node(
    #     package='tf2_ros',
    #     namespace='tf2',
    #     name='camera_to_base_link_transform',
    #     executable='static_transform_publisher',
    #     arguments=['-0.147499', '-0.0598990', '-0.238857', '0', '-0.34906585', '0', 'zed_front_base_link', 'base_link'],#x value differs from the report, but was experimentet to match better with this value.
    #     output='screen'  # Ensure logs are visible
    # )
    
    # static_transform_publisher_fimu = Node(
    #     package='tf2_ros',
    #     namespace='tf2',
    #     name='camera_to_base_link_transform',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'zed_imu_link', 'zed_front_base_link'],#x value differs from the report, but was experimentet to match better with this value.
    #     output='screen'  # Ensure logs are visible
    # )

    # static_transform_publisher_foptical = Node(
    #     package='tf2_ros',
    #     namespace='tf2',
    #     name='zed_to_base_tf',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'zed_front_base_link', 'zed_front_left_camera_optical_frame'],#x value differs from the report, but was experimentet to match better with this value.
    #     output='screen'  # Ensure logs are visible
    # )

  
    rtabmap_launch_dir = FindPackageShare('rtabmap_launch').find('rtabmap_launch')

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'rgb_topic': '/zed_front/zed/rgb_gray/image_rect_gray',
            'depth_topic': '/zed_front/zed/depth/depth_registered',
            'camera_info_topic': '/zed_front/zed/rgb_gray/camera_info',
            'depth_camera_info_topic': '/zed_front/zed/depth/camera_info',
            'imu_topic': '/zed_front/zed/imu/data',
            'frame_id': 'zed_camera_link',
            'approx_sync': 'true',
            'wait_imu_to_init': 'true'
        }.items()
    )

    delayed_actions = TimerAction(
    period=25.0,
    actions=[
        LogInfo(msg='Starting Rtab-slam after ZED camera initialization delay...'),

        
    ]
)
    # Create and return launch description
    return LaunchDescription([
        LogInfo(msg='Starting ZED2i Camera with RTAB-Map VSLAM...'),
        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='false',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false',   description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false',  description='Launch in localization mode.'),
        
        rtabmap_launch,
        # static_transform_publisher_fcam,
        # static_transform_publisher_fimu,
        # static_transform_publisher_foptical,
        delayed_actions    # Launch dependent nodes after a delay
    ])