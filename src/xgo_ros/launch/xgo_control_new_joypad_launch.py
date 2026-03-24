from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    root_pkg = get_package_share_directory('xgo2_ros')
    config_path = path.join(root_pkg, 'config', 'config.json')
    joy_profile_path = path.join(root_pkg, 'config', 'joy_newpad.yaml')

    use_joy = LaunchConfiguration('use_joy', default='true')
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joy',
            default_value='true',
            description='Launch joy_node and teleop_twist_joy',
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path',
        ),

        Node(
            package='xgo2_ros',
            executable='xgo2_ros_node',
            name='xgo2_ros_node',
            output='screen',
            parameters=[{'config_path': config_path}],
            emulate_tty=True,
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_profile_path, {'dev': joy_dev}],
            condition=IfCondition(use_joy),
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[joy_profile_path],
            condition=IfCondition(use_joy),
        ),
    ])
