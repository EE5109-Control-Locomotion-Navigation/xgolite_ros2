"""
xgolite_nav launch file
======================
Starts localization, A* planner, pure pursuit, orchestrator, and static base_link->robot_tag TF.

Transform chain:
  Camera tree: pi_camera -> tag36h11:0..4 (apriltag_ros), pi_camera -> workspace_origin (workspace_manager)
  Robot tree:  odom -> base_link (xgo_bt_node), base_link -> robot_tag (static)
  Bridge:      workspace_origin -> odom (localization_node, when tag36h11:4 visible)
  Full chain:  map/workspace_origin -> odom -> base_link -> robot_tag

Usage:
  ros2 launch xgolite_nav nav_launch.py

  # Override tag config for different setups:
  ros2 launch xgolite_nav nav_launch.py tag_family:=tag36h11 robot_tag_id:=5
  ros2 launch xgolite_nav nav_launch.py robot_tag_id:=7 tag_offset_z:=0.06

Requires: workspace_manager and camera/apriltag_ros running for TF and /workspace/boundary.
Requires: xgo2_ros_node for /cmd_vel and odom->base_link.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_defaults(params_file):
    """Load default values from nav_params.yaml."""
    with open(params_file) as f:
        cfg = yaml.safe_load(f) or {}
    tf_cfg = (cfg.get('static_tf') or {}).get('ros__parameters') or {}
    tf_cfg = (tf_cfg.get('base_link_to_tag') or {})
    loc_cfg = (cfg.get('localization_node') or {}).get('ros__parameters') or {}
    return {
        'tag_family': loc_cfg.get('tag_family', 'tag36h11'),
        'robot_tag_id': loc_cfg.get('robot_tag_id', 4),
        'tag_offset_x': tf_cfg.get('x', 0.0),
        'tag_offset_y': tf_cfg.get('y', 0.0),
        'tag_offset_z': tf_cfg.get('z', 0.05),
        'tag_offset_yaw': tf_cfg.get('yaw', 0.0),
        'tag_offset_pitch': tf_cfg.get('pitch', 0.0),
        'tag_offset_roll': tf_cfg.get('roll', 0.0),
    }


def _launch_nodes(context):
    pkg = get_package_share_directory('xgolite_nav')
    params_file = os.path.join(pkg, 'config', 'nav_params.yaml')
    defaults = _load_defaults(params_file)

    tag_family = LaunchConfiguration('tag_family', default=defaults['tag_family'])
    robot_tag_id = LaunchConfiguration('robot_tag_id', default=str(defaults['robot_tag_id']))
    tag_offset_x = LaunchConfiguration('tag_offset_x', default=str(defaults['tag_offset_x']))
    tag_offset_y = LaunchConfiguration('tag_offset_y', default=str(defaults['tag_offset_y']))
    tag_offset_z = LaunchConfiguration('tag_offset_z', default=str(defaults['tag_offset_z']))
    tag_offset_yaw = LaunchConfiguration('tag_offset_yaw', default=str(defaults['tag_offset_yaw']))
    tag_offset_pitch = LaunchConfiguration('tag_offset_pitch', default=str(defaults['tag_offset_pitch']))
    tag_offset_roll = LaunchConfiguration('tag_offset_roll', default=str(defaults['tag_offset_roll']))

    # Use 'robot_tag' as child frame to avoid conflict with apriltag_ros which publishes
    # pi_camera -> tag36h11:N. Both cannot use the same frame (tag36h11:4) as child.
    # robot_tag and tag36h11:4 represent the same physical point; localization uses
    # workspace_origin -> tag36h11:4 from camera + tag_to_base_link params.
    robot_tag_frame = 'robot_tag'

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_tag_static_tf',
        arguments=[
            tag_offset_x,
            tag_offset_y,
            tag_offset_z,
            tag_offset_yaw,
            tag_offset_pitch,
            tag_offset_roll,
            'base_link',
            robot_tag_frame,
        ],
    )

    return [
        static_tf_node,
        Node(
            package='xgolite_nav',
            executable='localization_node',
            name='localization_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'robot_tag_id': int(robot_tag_id.perform(context)),
                    'tag_family': tag_family.perform(context),
                },
            ],
        ),
        Node(
            package='xgolite_nav',
            executable='astar_planner_node',
            name='astar_planner_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
        Node(
            package='xgolite_nav',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
        Node(
            package='xgolite_nav',
            executable='nav_orchestrator_node',
            name='nav_orchestrator_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('xgolite_nav')
    params_file = os.path.join(pkg, 'config', 'nav_params.yaml')
    defaults = _load_defaults(params_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'tag_family',
            default_value=defaults['tag_family'],
            description='AprilTag family (e.g. tag36h11, tag25h9)',
        ),
        DeclareLaunchArgument(
            'robot_tag_id',
            default_value=str(defaults['robot_tag_id']),
            description='AprilTag ID mounted on the robot (must differ from workspace corner tags 0-3)',
        ),
        DeclareLaunchArgument(
            'tag_offset_x',
            default_value=str(defaults['tag_offset_x']),
            description='Base_link to tag offset X (m)',
        ),
        DeclareLaunchArgument(
            'tag_offset_y',
            default_value=str(defaults['tag_offset_y']),
            description='Base_link to tag offset Y (m)',
        ),
        DeclareLaunchArgument(
            'tag_offset_z',
            default_value=str(defaults['tag_offset_z']),
            description='Base_link to tag offset Z (m)',
        ),
        DeclareLaunchArgument(
            'tag_offset_yaw',
            default_value=str(defaults['tag_offset_yaw']),
            description='Base_link to tag offset yaw (rad)',
        ),
        DeclareLaunchArgument(
            'tag_offset_pitch',
            default_value=str(defaults['tag_offset_pitch']),
            description='Base_link to tag offset pitch (rad)',
        ),
        DeclareLaunchArgument(
            'tag_offset_roll',
            default_value=str(defaults['tag_offset_roll']),
            description='Base_link to tag offset roll (rad)',
        ),
        OpaqueFunction(function=lambda context: _launch_nodes(context)),
    ])
