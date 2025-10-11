import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    use_grayscale = LaunchConfiguration('use_grayscale')
    rgb_topic = LaunchConfiguration('rgb_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    # RTAB-Map launch directory
    rtabmap_launch_dir = FindPackageShare('rtabmap_launch').find('rtabmap_launch')

    # Include RTAB-Map launch file with conditional topics
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'compressed': 'true',
            'rtabmap_args': "--delete_db_on_start ",
            'rgb_topic': rgb_topic,
            'depth_topic': '/zed_front/zed/depth/depth_registered',
            'camera_info_topic': camera_info_topic,
            'frame_id': 'base_link',
            'approx_sync': 'true',
            'use_sim_time': 'true',
            'qos': '1',
            'topic_queue_size': '100',
            'sync_queue_size': '300',
            'rviz': LaunchConfiguration('rviz'),
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz')
        }.items()
    )

    # Delay RTAB-Map launch to allow camera initialization
    delayed_actions = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg='Starting Rtab-slam after ZED camera initialization delay...'),
        ]
    )

    return LaunchDescription([
        LogInfo(msg='Starting ZED2i Camera with RTAB-Map VSLAM...'),

        # Declare launch arguments
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('use_grayscale', default_value='false', description='Use grayscale input images'),

        # Conditionally set topics based on grayscale flag
        DeclareLaunchArgument(
            'rgb_topic',
            default_value=PythonExpression([
                "\"/zed_front/zed/rgb_gray/image_rect_gray\" if '", use_grayscale, "' == 'true' else \"/zed_front/zed/rgb/image_rect_color\""
            ]),
            description='RGB image topic'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value=PythonExpression([
                "\"/zed_front/zed/rgb_gray/camera_info\" if '", use_grayscale, "' == 'true' else \"/zed_front/zed/rgb/camera_info\""
            ]),
            description='Camera info topic'
        ),

        
        # Print selected topics
        LogInfo(msg=PythonExpression([
            "'Selected RGB topic: ' + '", rgb_topic, "'"
        ])),
        LogInfo(msg=PythonExpression([
            "'Selected Camera Info topic: ' + '", camera_info_topic, "'"
        ])),


        rtabmap_launch,
        delayed_actions
    ])