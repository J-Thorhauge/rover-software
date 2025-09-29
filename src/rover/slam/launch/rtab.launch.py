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
    
    pkg_stereo_image_proc = get_package_share_directory(
        'stereo_image_proc')

    # Paths
    stereo_image_proc_launch = PathJoinSubstitution(
        [pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py'])

    localization = LaunchConfiguration('localization')
    
    # 2. Launch sync node for ZED camera. This is from the old, another 
    # sync node has been added later IF THAT FUNCTIONS THEN DELETE THIS CODE
    # rgbd_sync_node = Node(
    #     package='rtabmap_sync',
    #     executable='rgbd_sync',
    #     output='screen',
    #     parameters=[{'approx_sync': True,
    #                 'approx_sync_max_interval': 0.05,
    #                 }],  
    #     remappings=[
    #         ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
    #         ('depth/image', '/zed/zed_node/depth/depth_registered'),
    #         ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
    #         ('rgbd_image', '/rgbd_image')
    #     ]
    # )
    
    # 3. Launch static transform publishers
    remaps=[
     ('left/image_rect',   'left_gray/image_rect_gray'),
     ('right/image_rect',  'right_gray/image_rect_gray'),
     ('left/camera_info',  'left_gray/camera_info'),
     ('right/camera_info', 'right_gray/camera_info'),
     ('rgbd_image', '/zed_front/zed/depth/depth_registered'),
     ('odom',       '/vo')]

    static_transform_publisher_1 = Node(
        package='tf2_ros',
        namespace='tf2',
        name='camera_to_base_link_transform',
        executable='static_transform_publisher',
        arguments=['-0.147499', '-0.0598990', '-0.238857', '0', '-0.34906585', '0', 'zed_camera_link', 'base_link'],#x value differs from the report, but was experimentet to match better with this value.
        output='screen'  # Ensure logs are visible
    )
    
    parameters ={
        'frame_id': 'base_link',
        'approx_sync': False,
        'map_negative_poses_ignored':True,
        'subscribe_odom_info': True, 
        'OdomF2M/MaxSize': '1000',
        'GFTT/MinDistance': '10',
        'GFTT/QualityLevel': '0.00001',
    }
  
    

    delayed_actions = TimerAction(
    period=25.0,
    actions=[
        LogInfo(msg='Starting Rtab-slam after ZED camera initialization delay...'),

        # Uncompress images for stereo_image_rect and remap to expected names from stereo_image_proc
        Node(
            package='image_transport', executable='republish', name='republish_left', output='screen',
            namespace='zed_front/zed',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', 'zed_front/zed/left_raw_gray/image_raw_gray/compressed'),
                        ('out',           'zed_front/zed/left_raw_gray/image_raw_gray')]
        ),
        Node(
            package='image_transport', executable='republish', name='republish_right', output='screen',
            namespace='zed_front/zed',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', 'zed_front/zed/right_raw_gray/image_raw_gray/compressed'),
                        ('out',           'zed_front/zed/right_raw_gray/image_raw_gray')]
        ),

        
        GroupAction(
            actions=[

                SetRemap(src='camera_info',dst='camera_info'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([stereo_image_proc_launch]),
                    launch_arguments=[
                        ('left_namespace', 'zed_front/zed/left'),
                        ('right_namespace', 'zed_front/zed/right'),
                        ('disparity_range', '128'),
                    ]
                ),
            ]
        ),

        Node(
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            namespace='zed_front/zed/',
            remappings=[
                ('left/image_rect',   'left_gray/image_rect_gray'),
                ('right/image_rect',  'right_gray/image_rect_gray'),
                ('left/camera_info',  'left_gray/camera_info'),
                ('right/camera_info', 'right_gray/camera_info')]
        ),

                # Visual odometry
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[parameters],
            remappings=remaps),
        
        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remaps,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remaps),


        # Node(
        #     package='rtabmap_viz',
        #     executable='rtabmap_viz',
        #     output='screen',
        #     parameters=[{
        #         'frame_id': 'base_link',
        #         'odom_frame_id': 'odom',
        #         'subscribe_rgbd': True,
        #         'subscribe_odom_info': True,
        #         'subscribe_scan_cloud': True,  # Optional: Enable for visualization
        #         'approx_sync': False
        #     }],
        #     remappings=[
        #         ('rgbd_image', '/rgbd_image'),
        #     ]
        # ),
    ]
)
    # Create and return launch description
    return LaunchDescription([
        LogInfo(msg='Starting ZED2i Camera with RTAB-Map VSLAM...'),
        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='false',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false',   description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false',  description='Launch in localization mode.'),
        
        SetParameter(name='use_sim_time', value=True),

        static_transform_publisher_1,
        delayed_actions    # Launch dependent nodes after a delay
    ])