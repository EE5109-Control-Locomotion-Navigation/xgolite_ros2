"""
AprilTag Workspace launch file
==============================
Starts the workspace_manager node and (optionally) an RViz2 instance.

Usage:
  ros2 launch apriltag_workspace workspace_launch.py
  ros2 launch apriltag_workspace workspace_launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('apriltag_workspace')
    params_file = os.path.join(pkg, 'config', 'workspace_params.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'workspace.rviz')

    use_rviz = LaunchConfiguration('rviz', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 for visualization',
        ),

        # ----------------------------------------------------------------
        # Workspace manager node
        # ----------------------------------------------------------------
        Node(
            package='apriltag_workspace',
            executable='workspace_manager',
            name='workspace_manager',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),

        # ----------------------------------------------------------------
        # RViz2  (conditional)
        # ----------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg] if os.path.isfile(rviz_cfg) else [],
            condition=IfCondition(use_rviz),
        ),
    ])
