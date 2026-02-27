#!/usr/bin/env python3
"""
Navigation Orchestrator Node
============================
Receives goals, calls A* planner, publishes path for pure pursuit to follow.
- Input: /move_base_simple/goal, /goal_pose (PoseStamped), /robot_pose (PoseStamped)
- Output: publishes to /plan (Path)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan


def _goal_qos() -> QoSProfile:
    """QoS that matches RViz2 2D Goal Pose (best-effort, volatile)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class NavOrchestratorNode(Node):
    """Wires goal -> plan -> pursue."""

    def __init__(self):
        super().__init__('nav_orchestrator_node')

        self.declare_parameter('map_frame', 'workspace_origin')
        self.declare_parameter('plan_tolerance', 0.5)

        self._map_frame = self.get_parameter('map_frame').value
        self._tolerance = self.get_parameter('plan_tolerance').value
        self._robot_pose: PoseStamped | None = None

        self._tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        self.create_subscription(PoseStamped, '/robot_pose', self._pose_cb, 10)
        self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self._goal_cb, _goal_qos()
        )
        self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, _goal_qos()
        )
        self._plan_pub = self.create_publisher(Path, '/plan', 10)
        self._client = self.create_client(GetPlan, '/plan_path')

        self.get_logger().info(
            'Nav orchestrator ready. Send goals to /move_base_simple/goal or /goal_pose'
        )

    def _pose_cb(self, msg: PoseStamped):
        self._robot_pose = msg

    def _transform_goal_to_map(self, msg: PoseStamped) -> PoseStamped | None:
        """Transform goal pose to map_frame. Returns None on failure."""
        if msg.header.frame_id == self._map_frame:
            return msg
        try:
            t = self._tf_buf.lookup_transform(
                self._map_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                Duration(seconds=0.5),
            )
            return do_transform_pose(msg, t)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().warn(f'Could not transform goal to {self._map_frame}: {e}')
            return None

    def _goal_cb(self, msg: PoseStamped):
        self.get_logger().info(
            f'Goal received (frame={msg.header.frame_id}, '
            f'x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f})'
        )

        if self._robot_pose is None:
            self.get_logger().warn('No robot pose yet. Waiting for localization.')
            return

        goal_map = self._transform_goal_to_map(msg)
        if goal_map is None:
            return

        if not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Plan service not available. Is astar_planner_node running?')
            return

        req = GetPlan.Request()
        req.start = self._robot_pose
        req.start.header.frame_id = self._map_frame
        req.goal = goal_map
        req.goal.header.frame_id = self._map_frame
        req.tolerance = self._tolerance

        future = self._client.call_async(req)
        future.add_done_callback(lambda f: self._plan_response(f, goal_map))

    def _plan_response(self, future, goal_msg: PoseStamped):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Plan request failed: {e}')
            return

        if not response.plan.poses:
            self.get_logger().warn('Planner returned empty path.')
            return

        response.plan.header.stamp = self.get_clock().now().to_msg()
        self._plan_pub.publish(response.plan)
        self.get_logger().info(
            f'Published plan with {len(response.plan.poses)} waypoints to goal.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
