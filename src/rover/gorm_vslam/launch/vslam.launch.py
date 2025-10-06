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
    
    
    camera_rotation_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_rotation_base_to_camera',
        # Format: x y z roll pitch yaw frame_id child_frame_id
        arguments=['0', '0', '0', '0', '-0.2618', '0', 'base_link', 'zed_camera_link'],
    )

    rtabmap_launch_dir = FindPackageShare('rtabmap_launch').find('rtabmap_launch')

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'compressed': 'true',
            'rtabmap_args': "--delete_db_on_start ",
            'rgb_topic': '/zed_front/zed/rgb_gray/image_rect_gray',
            'depth_topic': '/zed_front/zed/depth/depth_registered',
            'camera_info_topic': '/zed_front/zed/rgb_gray/camera_info',
            #'subscribe_rgbd': 'true',    # to be used if using the sync node 
            'frame_id': 'zed_camera_link',
            'approx_sync': 'true',
            'use_sim_time': 'true',
            #'wait_imu_to_init': 'true',
            #'imu_topic' :'/zed/zed_node/imu/data',
            'qos': '1',
            'topic_queue_size': '100',
            'sync_queue_size': '300',
            'rviz':'true', 
            'rtabmap_viz': 'true'
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
        #sync_node,
        rtabmap_launch,
        camera_rotation_tf,
        # static_transform_publisher_fcam,
        # static_transform_publisher_fimu,
        # static_transform_publisher_foptical,
        delayed_actions    # Launch dependent nodes after a delay
    ])