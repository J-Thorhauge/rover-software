import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    pkg_share = get_package_share_directory('gorm_nav2')
    robot_localization_file_path = os.path.join(pkg_share, 'params', 'ekf.yaml')


    # Paths to launch files
    nav2_bringup_path = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    yaml_name_arg = DeclareLaunchArgument(
        'config',
        default_value='rpp.yaml',
        description='Name of the yaml parameter file '
    )

    

    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('gorm_nav2'),
        'params',
        LaunchConfiguration('config')
        
    ])

    # Start robot localization using an Extended Kalman filter
    start_robot_localization = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path]
    )
    
    # Include the main Nav2 launch file with the parameter file
    # and add the remap for /cmd_vel -> /remote/cmd_vel
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_path),
        launch_arguments={
            'params_file': nav2_params_file,
            'cmd_vel_nav': '/remote/cmd_vel',  # Remap target topic       
        }.items()
    )

    return LaunchDescription([
        start_robot_localization,
        yaml_name_arg,
        nav2_launch
    ])
