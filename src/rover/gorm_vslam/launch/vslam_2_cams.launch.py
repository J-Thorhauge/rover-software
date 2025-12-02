import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
          
def generate_launch_description():     

    config_rviz = os.path.join(
            get_package_share_directory('rtabmap_examples'), 'config', 'slam_D405x2_config.rviz')         

    rviz_node = launch_ros.actions.Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=[["-d"], [config_rviz]]
    )

    rgbd_sync1_node = launch_ros.actions.Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync1', output="screen",
        parameters=[{
            "approx_sync": False
            }],
        remappings=[
            ("rgb/image", '/camera1/color/image_rect_raw'),
            ("depth/image", '/camera1/depth/image_rect_raw'),
            ("rgb/camera_info", '/camera1/color/camera_info'),
            ("rgbd_image", 'rgbd_image')],
        namespace='realsense_camera1'
    )
    rgbd_sync2_node = launch_ros.actions.Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync2', output="screen",
        parameters=[{
            "approx_sync": False
            }],
        remappings=[
            ("rgb/image", '/camera2/color/image_rect_raw'),
            ("depth/image", '/camera2/depth/image_rect_raw'),
            ("rgb/camera_info", '/camera2/color/camera_info'),
            ("rgbd_image", 'rgbd_image')],
        namespace='realsense_camera2'
    )
            
    # RGB-D odometry
    rgbd_odometry_node = launch_ros.actions.Node(
        package='rtabmap_odom', executable='rgbd_odometry', output="screen",
        parameters=[{
            "frame_id": 'base_link',
            "odom_frame_id": 'odom',
            "publish_tf": True,
            "approx_sync": True,
            "subscribe_rgbd": True,
            }],
        remappings=[
            ("rgbd_image", '/realsense_camera1/rgbd_image'),
            ("odom", 'odom')],
        arguments=["--delete_db_on_start", ''],
        prefix='',
        namespace='rtabmap'
    )

    # SLAM 
    slam_node = launch_ros.actions.Node(
        package='rtabmap_slam', executable='rtabmap', output="screen",
        parameters=[{
            "rgbd_cameras":2,
            "subscribe_depth": True,
            "subscribe_rgbd": True,
            "subscribe_rgb": True,
            "subscribe_odom_info": True,
            "frame_id": 'base_link',
            "map_frame_id": 'map',
            "publish_tf": True,
            "database_path": '~/.ros/rtabmap.db',
            "approx_sync": True,
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "true"
        }],
        remappings=[
            ("rgbd_image0", '/realsense_camera1/rgbd_image'),
            ("rgbd_image1", '/realsense_camera2/rgbd_image'),
            ("odom", 'odom')],
        arguments=["--delete_db_on_start"],
        prefix='',
        namespace='rtabmap'
    )

    voxelcloud1_node = launch_ros.actions.Node(
        package='rtabmap_util', executable='point_cloud_xyzrgb', name='point_cloud_xyzrgb1', output='screen',
        parameters=[{
            "approx_sync": True,
        }],
        remappings=[
            ('rgb/image', '/camera1/color/image_rect_raw'),
            ('depth/image', '/camera1/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera1/color/camera_info'),
            ('rgbd_image', 'rgbd_image'),
            ('cloud', 'voxel_cloud1')]
    )

    voxelcloud2_node = launch_ros.actions.Node(
        package='rtabmap_util', executable='point_cloud_xyzrgb', name='point_cloud_xyzrgb2', output='screen',
        parameters=[{
            "approx_sync": True,
        }],
        remappings=[
            ('rgb/image', '/camera2/color/image_rect_raw'),
            ('depth/image', '/camera2/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera2/color/camera_info'),
            ('rgbd_image', 'rgbd_image'),
            ('cloud', 'voxel_cloud2')]
    )    

    return launch.LaunchDescription(
        [
        rviz_node,
        rgbd_sync1_node,
        rgbd_sync2_node,
        rgbd_odometry_node,
        slam_node,
        voxelcloud1_node,
        voxelcloud2_node
        ]