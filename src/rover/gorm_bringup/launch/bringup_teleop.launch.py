
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # cmd_vel to motor commands converter
    ackermann_nav2 = Node(
        package='gorm_base_control',
        executable='ackermann_nav2',
        name='ackermann_nav2',
        output='screen'
    )

    motor_driver_node = Node(
        package='gorm_base_control',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='screen'
    )

    # Joy node
    joy_node = Node(
        package='joy',
        namespace='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # Joy to cmd_vel converter (if reimplemented in gorm_teleop)
    joy_to_cmd_vel_node = Node(
        package='gorm_teleop',
        executable='joy_to_cmd_vel_node',
        name='joy_to_cmd_vel_node',
        output='screen'
    )

    # Web interface (includes rosbridge and web server)
    web_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gorm_web_interface'), 'launch'),
            '/web_interface.launch.py'
        ])
    )

    ld.add_action(joy_node)
    ld.add_action(joy_to_cmd_vel_node)
    ld.add_action(ackermann_nav2)
    ld.add_action(motor_driver_node)
    ld.add_action(web_interface)

    return ld
