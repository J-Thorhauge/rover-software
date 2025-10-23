import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Paths to launch files
    nav2_bringup_path = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('gorm_nav2'),
        'params',
        'gorm_nav2.yaml'
    ])
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_path),
        launch_arguments=[
            ('params_file', nav2_params_file)
        ]
    )

    return LaunchDescription([
        nav2_launch
    ])