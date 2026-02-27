#!/usr/bin/env python3
"""
Pure Pursuit Node
=================
Follows a path using pure pursuit control.
- Input: /plan (Path), /robot_pose (PoseStamped)
- Output: /cmd_vel (Twist)
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitNode(Node):
    """Pure pursuit path follower for holonomic robot."""

    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('lookahead_distance', 0.15)
        self.declare_parameter('max_linear_velocity', 0.25)
        self.declare_parameter('max_angular_velocity', 1.2)
        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('control_rate_hz', 20.0)

        self._lookahead = self.get_parameter('lookahead_distance').value
        self._max_linear = self.get_parameter('max_linear_velocity').value
        self._max_angular = self.get_parameter('max_angular_velocity').value
        self._goal_tolerance = self.get_parameter('goal_tolerance').value
        rate = self.get_parameter('control_rate_hz').value

        self._path: Path | None = None
        self._robot_pose: PoseStamped | None = None

        self.create_subscription(Path, '/plan', self._path_cb, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self._pose_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._timer = self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            f'Pure pursuit ready | lookahead={self._lookahead}m | goal_tolerance={self._goal_tolerance}m'
        )

    def _path_cb(self, msg: Path):
        self._path = msg if len(msg.poses) > 0 else None

    def _pose_cb(self, msg: PoseStamped):
        self._robot_pose = msg

    def _update(self):
        if self._path is None or self._robot_pose is None:
            return

        rx = self._robot_pose.pose.position.x
        ry = self._robot_pose.pose.position.y
        rq = self._robot_pose.pose.orientation
        rtheta = _yaw_from_quaternion(rq.x, rq.y, rq.z, rq.w)

        poses = self._path.poses
        if not poses:
            self._stop()
            return

        goal_x = poses[-1].pose.position.x
        goal_y = poses[-1].pose.position.y
        dist_to_goal = math.hypot(goal_x - rx, goal_y - ry)

        if dist_to_goal < self._goal_tolerance:
            self._stop()
            return

        # Find lookahead point: first point on path at or beyond lookahead_distance from robot
        lookahead_x = goal_x
        lookahead_y = goal_y
        for ps in poses:
            px = ps.pose.position.x
            py = ps.pose.position.y
            d = math.hypot(px - rx, py - ry)
            if d >= self._lookahead:
                lookahead_x = px
                lookahead_y = py
                break

        dx = lookahead_x - rx
        dy = lookahead_y - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            self._stop()
            return

        # Direction to lookahead in world frame (unit vector)
        ux = dx / dist
        uy = dy / dist

        # Transform to robot body frame: body x = forward, body y = left
        # World to body: rotate by -theta
        cos_t = math.cos(rtheta)
        sin_t = math.sin(rtheta)
        vx_body = ux * cos_t + uy * sin_t
        vy_body = -ux * sin_t + uy * cos_t

        # Scale by max velocity; reduce speed when close to goal
        scale = min(1.0, dist_to_goal / self._lookahead)
        v = self._max_linear * scale
        vx = v * vx_body
        vy = v * vy_body

        # Angular velocity: turn toward lookahead
        alpha = math.atan2(dy, dx) - rtheta
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi
        vyaw = 2.0 * v * math.sin(alpha) / max(self._lookahead, 0.01)
        vyaw = max(-self._max_angular, min(self._max_angular, vyaw))

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(vyaw)
        self._cmd_pub.publish(cmd)

    def _stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
