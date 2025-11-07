import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # Paths to launch files
    nav2_bringup_path = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    yaml_name_arg = DeclareLaunchArgument(
        'config',
        default_value='mppi.yaml',
        description='Name of the yaml parameter file '
    )

    

    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('gorm_nav2'),
        'params',
        LaunchConfiguration('config')
        
    ])

    # Launch the Ackermann BT control servers
    steering_server = Node(
        package='ackermann_bt_control',
        executable='steering_action_server',
        name='steering_action_server',
        output='screen'
    )

    rotate_server = Node(
        package='ackermann_bt_control',
        executable='rotation_action_server',
        name='rotation_action_server',
        output='screen'
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
        yaml_name_arg,
        nav2_launch
    ])
