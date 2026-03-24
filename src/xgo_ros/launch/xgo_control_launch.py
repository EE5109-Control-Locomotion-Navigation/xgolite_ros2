from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    root_pkg = get_package_share_directory('xgo2_ros')
    config_path = path.join(root_pkg, 'config', 'config.json')
    use_joy = LaunchConfiguration('use_joy', default='true')
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')
    joypad_profile = LaunchConfiguration('joypad_profile', default='classic')
    right_stick_h_axis = LaunchConfiguration('right_stick_h_axis', default='3')
    right_stick_v_axis = PythonExpression(["'2' if '", joypad_profile, "' == 'newpad' else '4'"])
    walk_yaw_axis = PythonExpression(["'2' if '", joypad_profile, "' == 'newpad' else '3'"])
    teleop_yaw_axis = PythonExpression(["2 if '", joypad_profile, "' == 'newpad' else 3"])
    walk_button_index = LaunchConfiguration('walk_button_index')
    pose_button_index = LaunchConfiguration('pose_button_index')
    tl2_button_index = LaunchConfiguration('tl2_button_index')
    tl2_axis_index = LaunchConfiguration('tl2_axis_index')
    pose_swap_roll_pitch = LaunchConfiguration('pose_swap_roll_pitch')
    pose_roll_sign = LaunchConfiguration('pose_roll_sign')

    return LaunchDescription([
        DeclareLaunchArgument('use_joy', default_value='true',
                              description='Launch joy_node and teleop_twist_joy'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0',
                              description='Joystick device path'),
        DeclareLaunchArgument('joypad_profile', default_value='classic',
                              description="Joystick profile: 'classic' or 'newpad'"),
        DeclareLaunchArgument('right_stick_h_axis', default_value='3',
                              description='Right-stick horizontal axis index (xgo node)'),
        DeclareLaunchArgument('walk_button_index',
                              default_value=PythonExpression(["'4' if '", joypad_profile, "' == 'newpad' else '4'"]),
                              description='TL1 button index'),
        DeclareLaunchArgument('pose_button_index',
                              default_value=PythonExpression(["'5' if '", joypad_profile, "' == 'newpad' else '5'"]),
                              description='TR1 button index'),
        DeclareLaunchArgument('tl2_button_index',
                              default_value=PythonExpression(["'6' if '", joypad_profile, "' == 'newpad' else '-1'"]),
                              description='TL2 button index (-1 to use trigger axis instead)'),
        DeclareLaunchArgument('tl2_axis_index',
                              default_value=PythonExpression(["'-1' if '", joypad_profile, "' == 'newpad' else '2'"]),
                              description='TL2 trigger axis index (-1 to disable axis-based TL2)'),
        DeclareLaunchArgument('pose_swap_roll_pitch',
                              default_value=PythonExpression(["'true' if '", joypad_profile, "' == 'newpad' else 'false'"]),
                              description='Swap right-stick roll/pitch assignment in pose/attitude modes'),
        DeclareLaunchArgument('pose_roll_sign',
                              default_value=PythonExpression(["'-1.0' if '", joypad_profile, "' == 'newpad' else '1.0'"]),
                              description='Sign multiplier for roll in pose/attitude modes'),

        Node(
            package='xgo2_ros',
            executable='xgo2_ros_node',
            name='xgo2_ros_node',
            output='screen',
            parameters=[{
                'config_path': config_path,
                'right_stick_h_axis': right_stick_h_axis,
                'right_stick_v_axis': right_stick_v_axis,
                'walk_yaw_axis': walk_yaw_axis,
                'walk_yaw_sign': 1.0,
                'joypad_profile': joypad_profile,
                'walk_button_index': walk_button_index,
                'pose_button_index': pose_button_index,
                'tl2_button_index': tl2_button_index,
                'tl2_axis_index': tl2_axis_index,
                'pose_swap_roll_pitch': pose_swap_roll_pitch,
                'pose_roll_sign': pose_roll_sign,
            }],
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
                'axis_angular.yaw': teleop_yaw_axis,
                'scale_linear.x':   0.3,
                'scale_linear.y':   0.2,
                'scale_angular.yaw': 1.2,
                'enable_button': 4,    # LB – hold to enable motion
            }],
            condition=IfCondition(use_joy),
        ),
    ])
