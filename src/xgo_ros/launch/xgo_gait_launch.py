from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bt_address',
                              default_value='38:18:2B:4F:E9:DA'),
        DeclareLaunchArgument('lib_path',
                              default_value='/home/brian/EE5109/xgo_gait'),
        DeclareLaunchArgument('gait', default_value='crawl',
                              description='crawl | trot | pace'),
        DeclareLaunchArgument('mode', default_value='joint',
                              description='joint (0x50) | foot (0x40)'),
        DeclareLaunchArgument('hz', default_value='20.0'),
        DeclareLaunchArgument('stand_z', default_value='95.0'),
        DeclareLaunchArgument('calibrate', default_value=''),
        Node(
            package='xgo2_ros',
            executable='xgo_gait_node',
            name='xgo_gait_node',
            output='screen',
            parameters=[{
                'bt_address': LaunchConfiguration('bt_address'),
                'lib_path': LaunchConfiguration('lib_path'),
                'gait': LaunchConfiguration('gait'),
                'mode': LaunchConfiguration('mode'),
                'hz': LaunchConfiguration('hz'),
                'stand_z': LaunchConfiguration('stand_z'),
                'calibrate': LaunchConfiguration('calibrate'),
            }],
            emulate_tty=True,
        ),
    ])
