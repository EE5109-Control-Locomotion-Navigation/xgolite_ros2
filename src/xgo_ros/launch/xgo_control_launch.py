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
    use_joy = LaunchConfiguration('use_joy', default='true')
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')

    return LaunchDescription([
        DeclareLaunchArgument('use_joy', default_value='true',
                              description='Launch joy_node and teleop_twist_joy'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0',
                              description='Joystick device path'),

        Node(
            package='xgo2_ros',
            executable='xgo2_ros_node',
            name='xgo2_ros_node',
            output='screen',
            parameters=[{'config_path': config_path}],
            emulate_tty=True,
        ),

        # Joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': joy_dev, 'autorepeat_rate': 20.0}],
            condition=IfCondition(use_joy),
        ),

        # Teleop: maps Joy → cmd_vel
        # Hold the enable button (default button 0 / A on Xbox) to move
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[{
                'axis_linear.x':  1,   # left stick vertical
                'axis_linear.y':  0,   # left stick horizontal
                'axis_angular.yaw': 3, # right stick horizontal
                'scale_linear.x':   0.3,
                'scale_linear.y':   0.2,
                'scale_angular.yaw': 1.2,
                'enable_button': 4,    # LB – hold to enable motion
            }],
            condition=IfCondition(use_joy),
        ),
    ])
