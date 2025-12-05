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
    # Launch configuration variables
    localization = LaunchConfiguration('localization')
    rtabmap_args = LaunchConfiguration('rtabmap_args')
    use_grayscale = LaunchConfiguration('use_grayscale')
    rgb_topic = LaunchConfiguration('rgb_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    config_file = os.path.join(
    FindPackageShare('gorm_vslam').find('gorm_vslam'),
    'config',
    'rtabmap_params.yaml'
    )
    
    #Open the config files
    with open(config_file,'r') as file:
        loaded_parameters = yaml.safe_load(file)
    #Standard ros2 config files are nested so we grab just the ros parameters to ensure we get the correct data form
    launch_args = loaded_parameters["rtabmap"]["ros__parameters"]

    
    # Add flexible launch configurations
    launch_args['rviz'] = LaunchConfiguration('rviz')
    launch_args['rtabmap_viz'] = LaunchConfiguration('rtabmap_viz')
    #launch_args['localization'] = LaunchConfiguration('localization')



    # RTAB-Map launch directory
    rtabmap_launch_dir = FindPackageShare('rtabmap_launch').find('rtabmap_launch')

    map_db_path ='/home/roy/Documents/vslam_maps/inside_recording.db'

    stereo_namespace = '/zed_front/zed'
    left_image = f'{stereo_namespace}/left/image_rect_color'
    right_image = f'{stereo_namespace}/right/image_rect_color'
    left_info = f'{stereo_namespace}/left/camera_info'
    right_info = f'{stereo_namespace}/right/camera_info'

    map_republisher = Node(
    package='gorm_vslam',
    executable='map_republisher.py',
    name='map_republisher',
    output='screen'
    )


    # Include RTAB-Map launch file with conditional topics
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments=launch_args.items()
    )

    # Delay RTAB-Map launch to allow camera initialization
    delayed_actions = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg='Starting Rtab-slam after ZED camera initialization delay...'),
        ]
    )

    pointcloud_converter = Node(
        package='elevation_map',
        executable='pointcloud_converter',
        name='pointcloud_converter',
        output='screen'
    )


    vslam_config_folder = os.path.join(get_package_share_directory("gorm_vslam"), "config")
    map_alignment_path = os.path.join(vslam_config_folder, "map_alignment.yaml")

    
    # aligner_node = Node(
    #     package='gorm_vslam', 
    #     executable='world_map_aligner.py',
    #     name='world_map_aligner',
    #     output='screen',
    #     parameters=[map_alignment_path],
    #     # Env var helpful to see TF warnings/time sync
    #     emulate_tty=True,
    # )


    aligner_node = Node(
        package='gorm_mapping',
        executable='world_map_aligner',
        name='world_map_aligner',
        output='screen',
        # If you use a YAML file:
        #['/home/jonas/rover-software/src/rover/gorm_mapping/config/map_alignment.yaml'],
        # parameters=[map_alignment_path],
        parameters=[{   # [map_alignment_path] if map_alignment_path.perform(None) != '' else 
            # Inline parameters (works without YAML; adjust as needed)
            'world_frame': 'world',
            'map_frame': 'map',
            'marker_frame_prefix': 'aruco_marker_',
            'min_markers': 2,
            'recompute': True,
            'timer_period': 1.0,
            'known_markers_json': '{"51":{"x":0.5,"y":3.0},"52":{"x":0.5,"y":6.0},"53":{"x":-1.0,"y":6.0},"54":{"x":-0.5,"y":9.0},"55":{"x":0.5,"y":12.0},"56":{"x":0.0,"y":13.0}}'
            # 'known_markers_json': '{"1":{"x":2.0,"y":0.5},"2":{"x":5.0,"y":0.5},"7":{"x":2.0,"y":3.0}}'
            # 'known_markers': {
            #     '1': {'x': 2.0, 'y': 0.5},
            #     '2': {'x': 5.0, 'y': 0.5},
            #     '7': {'x': 2.0, 'y': 3.0},
            # },
        }],
        # Env var helpful to see TF warnings/time sync
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




    static_transform = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='tf_map_to_odom',
    arguments=[
        '--x', '1.0',        # translation (m)
        '--y', '0.0',
        '--z', '0.0',
        '--yaw', '0.0',      # rotation (radians)
        '--pitch', '0.0',
        '--roll', '0.0',
        '--frame-id', 'map',          # parent frame
        '--child-frame-id', 'odom'  # child frame
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

        #Conditionally set if rtabmap.db should be deleted.
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value=PythonExpression([
                "\"--delete_db_on_start\" if '",localization, "'=='false' else ''"
            ]),
            description ='Deletes the previous map if localization is off'
        ),

        
        # Print selected topics
        LogInfo(msg=PythonExpression([
            "'Selected RGB topic: ' + '", rgb_topic, "'"
        ])),
        LogInfo(msg=PythonExpression([
            "'Selected Camera Info topic: ' + '", camera_info_topic, "'"
        ])),

        #static_transform,
        # rtabmap_launch,
        # pointcloud_converter,
        map_republisher,
        aligner_node,
        aruco_tf_node
        # delayed_actions
    ])