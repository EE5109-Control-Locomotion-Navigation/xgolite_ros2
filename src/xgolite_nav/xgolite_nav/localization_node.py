#!/usr/bin/env python3
"""
Localization Node
=================
Provides robot pose from AprilTag and occupancy grid from workspace boundary.
Connects camera TF tree to robot: publishes workspace_origin -> odom so that
map -> odom -> base_link is a full chain for RViz and nav tools.
- Input: TF (workspace_origin, robot AprilTag, odom->base_link), /workspace/boundary
- Output: /robot_pose (PoseStamped), /map (OccupancyGrid), TF workspace_origin->odom
"""

import math

import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_geometry_msgs import do_transform_point, do_transform_pose

from geometry_msgs.msg import (
    Point,
    PointStamped,
    PolygonStamped,
    Pose,
    PoseStamped,
    TransformStamped,
)
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from xgolite_nav.grid_utils import polygon_to_occupancy_grid


class LocalizationNode(Node):
    """AprilTag-based localization and map from workspace boundary."""

    def __init__(self):
        super().__init__('localization_node')

        self.declare_parameter('map_frame', 'workspace_origin')
        self.declare_parameter('publish_map_frame', True)  # publish static map->workspace_origin
        self.declare_parameter('camera_frame', 'pi_camera')
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('robot_tag_id', 4)
        self.declare_parameter('tag_to_base_link_x', 0.0)
        self.declare_parameter('tag_to_base_link_y', 0.0)
        self.declare_parameter('tag_to_base_link_z', 0.0)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_inflation_radius', 0.05)
        self.declare_parameter('publish_rate_hz', 10.0)

        self._map_frame = self.get_parameter('map_frame').value
        self._publish_map_frame = self.get_parameter('publish_map_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value
        self._family = self.get_parameter('tag_family').value
        self._robot_tag_id = int(self.get_parameter('robot_tag_id').value)
        self._tag_offset = (
            self.get_parameter('tag_to_base_link_x').value,
            self.get_parameter('tag_to_base_link_y').value,
            self.get_parameter('tag_to_base_link_z').value,
        )
        self._map_resolution = self.get_parameter('map_resolution').value
        self._inflation = self.get_parameter('map_inflation_radius').value
        rate = self.get_parameter('publish_rate_hz').value

        self._robot_tag_frame = f'{self._family}:{self._robot_tag_id}'

        self._tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._pub_pose = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self._pub_map = self.create_publisher(OccupancyGrid, '/map', 10)

        self._boundary_polygon_xy: list[tuple[float, float]] | None = None
        self._map_msg: OccupancyGrid | None = None

        self.create_subscription(PolygonStamped, '/workspace/boundary', self._boundary_cb, 10)
        self._timer = self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            f'Localization ready | map_frame={self._map_frame} | robot_tag={self._robot_tag_frame}'
        )

    def _boundary_cb(self, msg: PolygonStamped):
        """Transform boundary from camera frame to map frame and build occupancy grid."""
        if not msg.polygon.points:
            return

        try:
            t = self._tf_buf.lookup_transform(
                self._map_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                Duration(seconds=0.1),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Could not transform boundary to map frame', throttle_duration_sec=2.0)
            return

        polygon_xy = []
        for pt in msg.polygon.points:
            ps = PointStamped(header=msg.header, point=Point(x=pt.x, y=pt.y, z=pt.z))
            pt_map = do_transform_point(ps, t)
            polygon_xy.append((pt_map.point.x, pt_map.point.y))

        if len(polygon_xy) >= 3:
            self._boundary_polygon_xy = polygon_xy
            self._publish_map()

    def _publish_map(self):
        if self._boundary_polygon_xy is None:
            return

        grid_2d, ox, oy = polygon_to_occupancy_grid(
            self._boundary_polygon_xy,
            self._map_resolution,
            self._inflation,
        )
        rows = len(grid_2d)
        cols = len(grid_2d[0]) if rows else 0
        data = []
        for row in grid_2d:
            data.extend(row)

        msg = OccupancyGrid()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self._map_resolution
        msg.info.width = cols
        msg.info.height = rows
        msg.info.origin.position.x = ox
        msg.info.origin.position.y = oy
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data

        self._map_msg = msg
        self._pub_map.publish(msg)

    def _update(self):
        """Publish robot pose from TF."""
        try:
            t = self._tf_buf.lookup_transform(
                self._map_frame,
                self._robot_tag_frame,
                rclpy.time.Time(),
                Duration(seconds=0.1),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        # Apply tag-to-base_link offset (in tag frame: x forward, y left, z up)
        # For 2D nav we mainly care about x,y; z offset adjusts height.
        dx, dy, dz = self._tag_offset
        q = t.transform.rotation
        c = 2 * (q.w * q.w + q.z * q.z) - 1
        s = 2 * (q.w * q.x - q.y * q.z)
        # Rotate offset by tag yaw (assuming tag is upright, only yaw matters in 2D)
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        ox = dx * cos_y - dy * sin_y
        oy = dx * sin_y + dy * cos_y

        msg = PoseStamped()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._map_frame)
        msg.pose.position.x = t.transform.translation.x + ox
        msg.pose.position.y = t.transform.translation.y + oy
        msg.pose.position.z = t.transform.translation.z + dz
        msg.pose.orientation = t.transform.rotation

        self._pub_pose.publish(msg)

        # Publish workspace_origin -> odom to connect camera tree to robot tree.
        # T_map_odom = T_map_base * T_base_odom so that map->odom->base_link = map->base
        try:
            t_base_odom = self._tf_buf.lookup_transform(
                'base_link', 'odom', rclpy.time.Time(), Duration(seconds=0.1)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        # Transform odom origin through base to map: odom(0,0,0) -> base -> map
        origin_odom = PointStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id='odom'),
            point=Point(x=0.0, y=0.0, z=0.0),
        )
        origin_base = do_transform_point(origin_odom, t_base_odom)

        ts_map_base = TransformStamped()
        ts_map_base.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._map_frame)
        ts_map_base.child_frame_id = 'base_link'
        ts_map_base.transform.translation.x = msg.pose.position.x
        ts_map_base.transform.translation.y = msg.pose.position.y
        ts_map_base.transform.translation.z = msg.pose.position.z
        ts_map_base.transform.rotation = msg.pose.orientation

        origin_base_stamped = PointStamped(
            header=Header(stamp=origin_base.header.stamp, frame_id='base_link'),
            point=origin_base.point,
        )
        origin_map = do_transform_point(origin_base_stamped, ts_map_base)

        # Rotation: compose map->base and base->odom
        identity = Pose()
        identity.orientation.w = 1.0
        pose_odom = PoseStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id='odom'),
            pose=identity,
        )
        pose_base = do_transform_pose(pose_odom, t_base_odom)
        pose_map = do_transform_pose(
            PoseStamped(header=Header(frame_id='base_link'), pose=pose_base.pose),
            ts_map_base,
        )

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self._map_frame
        ts.child_frame_id = 'odom'
        ts.transform.translation.x = origin_map.point.x
        ts.transform.translation.y = origin_map.point.y
        ts.transform.translation.z = origin_map.point.z
        ts.transform.rotation = pose_map.pose.orientation
        self._tf_broadcaster.sendTransform(ts)

        # Optional: publish map -> workspace_origin (identity) for RViz fixed frame
        if self._publish_map_frame and self._map_frame == 'workspace_origin':
            ts_map = TransformStamped()
            ts_map.header.stamp = self.get_clock().now().to_msg()
            ts_map.header.frame_id = 'map'
            ts_map.child_frame_id = 'workspace_origin'
            ts_map.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(ts_map)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
