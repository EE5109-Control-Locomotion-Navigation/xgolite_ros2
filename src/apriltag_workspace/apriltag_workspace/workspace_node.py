#!/usr/bin/env python3
"""
AprilTag Workspace Manager Node
================================
Listens for TF transforms published by the remote pi camera:
    pi_camera  →  tag36h11:0
    pi_camera  →  tag36h11:1
    pi_camera  →  tag36h11:2
    pi_camera  →  tag36h11:3

For each visible tag:
  • Publishes a sphere + XYZ-axes triad + text label in RViz.
  • Publishes PoseArray on /workspace/tag_poses.

When all 4 boundary tags are visible:
  • Orders them CCW in the camera X-Y plane.
  • Publishes PolygonStamped on /workspace/boundary.
  • Draws a cyan LINE_STRIP around the perimeter.
  • Draws a semi-transparent filled quad.
  • Broadcasts a dynamic 'workspace_origin' TF at the centroid of the 4 tags.
    This frame later serves as the fixed reference for navigation.

Parameters (workspace_params.yaml):
  camera_frame       : source TF frame  (default: 'pi_camera')
  tag_family         : AprilTag family  (default: 'tag36h11')
  tag_ids            : list of 4 IDs   (default: [0, 1, 2, 3])
  tag_size_m         : physical size    (default: 0.07)
  publish_rate_hz    : loop rate        (default: 5.0)
  tf_timeout_s       : TF lookup limit  (default: 0.1)
"""

import math

import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformBroadcaster

from builtin_interfaces.msg import Duration as DurMsg
from geometry_msgs.msg import (
    Point,
    Point32,
    PolygonStamped,
    Pose,
    PoseArray,
    TransformStamped,
)
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class WorkspaceManagerNode(Node):
    """Maintains the workspace polygon from 4 AprilTag corner markers."""

    def __init__(self):
        super().__init__('workspace_manager')

        # ---- Parameters ------------------------------------------------
        self.declare_parameter('camera_frame', 'pi_camera')
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_ids', [0, 1, 2, 3])
        self.declare_parameter('tag_size_m', 0.07)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('tf_timeout_s', 0.1)

        self._cam = self.get_parameter('camera_frame').value
        self._family = self.get_parameter('tag_family').value
        self._ids = list(self.get_parameter('tag_ids').value)
        self._tag_size = float(self.get_parameter('tag_size_m').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._tf_timeout = float(self.get_parameter('tf_timeout_s').value)

        self._tag_frames = [f'{self._family}:{i}' for i in self._ids]

        # ---- TF --------------------------------------------------------
        self._tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)
        self._broadcaster = TransformBroadcaster(self)

        # ---- Publishers ------------------------------------------------
        self._pub_markers = self.create_publisher(
            MarkerArray, '/workspace/markers', 10)
        self._pub_poses = self.create_publisher(
            PoseArray, '/workspace/tag_poses', 10)
        self._pub_boundary = self.create_publisher(
            PolygonStamped, '/workspace/boundary', 10)

        # ---- State -----------------------------------------------------
        # Maps tag_frame -> latest TransformStamped
        self._transforms: dict[str, TransformStamped] = {}

        # ---- Timer -----------------------------------------------------
        self._timer = self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            f'WorkspaceManager ready | camera={self._cam} | '
            f'tags={self._tag_frames}'
        )

    # ------------------------------------------------------------------
    # Update loop
    # ------------------------------------------------------------------

    def _update(self):
        now_t = rclpy.time.Time()  # 0 = "latest available"
        timeout = Duration(seconds=self._tf_timeout)

        for frame in self._tag_frames:
            try:
                tf = self._tf_buf.lookup_transform(
                    self._cam, frame, now_t, timeout)
                self._transforms[frame] = tf
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass  # tag not yet visible – keep last known pose if any

        if not self._transforms:
            return

        self._publish_poses()
        self._publish_markers()

        if len(self._transforms) == len(self._tag_frames):
            self._publish_boundary()
            self._broadcast_workspace_origin()

        visible = len(self._transforms)
        total = len(self._tag_frames)
        if visible < total:
            self.get_logger().debug(
                f'Visible tags: {visible}/{total}', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _xyz(self, tf: TransformStamped) -> tuple[float, float, float]:
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def _to_pose(self, tf: TransformStamped) -> Pose:
        p = Pose()
        p.position.x = tf.transform.translation.x
        p.position.y = tf.transform.translation.y
        p.position.z = tf.transform.translation.z
        p.orientation = tf.transform.rotation
        return p

    def _order_ccw(self) -> list[str]:
        """Sort frames CCW by angle from centroid in the camera X-Y plane."""
        pos = {f: self._xyz(self._transforms[f]) for f in self._tag_frames}
        cx = sum(p[0] for p in pos.values()) / len(pos)
        cy = sum(p[1] for p in pos.values()) / len(pos)
        return sorted(
            self._tag_frames,
            key=lambda f: math.atan2(pos[f][1] - cy, pos[f][0] - cx),
        )

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_poses(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._cam
        for frame in self._tag_frames:
            if frame in self._transforms:
                msg.poses.append(self._to_pose(self._transforms[frame]))
        self._pub_poses.publish(msg)

    def _publish_boundary(self):
        ordered = self._order_ccw()
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._cam
        for frame in ordered:
            x, y, z = self._xyz(self._transforms[frame])
            msg.polygon.points.append(Point32(x=float(x), y=float(y), z=float(z)))
        self._pub_boundary.publish(msg)

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        lifetime = DurMsg(sec=2, nanosec=0)
        ma = MarkerArray()
        mid = 0

        # Per-tag colour palette
        palette = [
            ColorRGBA(r=1.0, g=0.25, b=0.25, a=1.0),   # red
            ColorRGBA(r=0.25, g=1.0, b=0.25, a=1.0),   # green
            ColorRGBA(r=0.35, g=0.35, b=1.0, a=1.0),   # blue
            ColorRGBA(r=1.0, g=1.0, b=0.0,  a=1.0),   # yellow
        ]

        for i, frame in enumerate(self._tag_frames):
            if frame not in self._transforms:
                continue
            tf = self._transforms[frame]
            x, y, z = self._xyz(tf)
            col = palette[i % len(palette)]

            # --- Sphere at tag centre ---
            sp = Marker()
            sp.header.stamp = now
            sp.header.frame_id = self._cam
            sp.ns = 'tag_spheres'
            sp.id = mid; mid += 1
            sp.type = Marker.SPHERE
            sp.action = Marker.ADD
            sp.pose.position.x = x
            sp.pose.position.y = y
            sp.pose.position.z = z
            sp.pose.orientation.w = 1.0
            d = self._tag_size * 2.0
            sp.scale.x = sp.scale.y = sp.scale.z = d
            sp.color = col
            sp.lifetime = lifetime
            ma.markers.append(sp)

            # --- XYZ axes triad in the tag's own frame ---
            # Expressed in the tag frame so they rotate with the tag.
            axis_defs = [
                (Point(x=self._tag_size, y=0.0, z=0.0),
                 ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)),   # X – red
                (Point(x=0.0, y=self._tag_size, z=0.0),
                 ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)),   # Y – green
                (Point(x=0.0, y=0.0, z=self._tag_size),
                 ColorRGBA(r=0.0, g=0.4, b=1.0, a=1.0)),   # Z – blue
            ]
            for tip, ax_col in axis_defs:
                arr = Marker()
                arr.header.stamp = now
                arr.header.frame_id = frame   # tag's own frame
                arr.ns = f'tag_axes_{self._ids[i]}'
                arr.id = mid; mid += 1
                arr.type = Marker.ARROW
                arr.action = Marker.ADD
                arr.points = [Point(x=0.0, y=0.0, z=0.0), tip]
                arr.scale.x = 0.006   # shaft diameter
                arr.scale.y = 0.014   # head diameter
                arr.scale.z = 0.0
                arr.color = ax_col
                arr.lifetime = lifetime
                ma.markers.append(arr)

            # --- Text label ---
            txt = Marker()
            txt.header.stamp = now
            txt.header.frame_id = self._cam
            txt.ns = 'tag_labels'
            txt.id = mid; mid += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = x
            txt.pose.position.y = y
            txt.pose.position.z = z - self._tag_size * 3.0
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.08
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = f'ID {self._ids[i]}'
            txt.lifetime = lifetime
            ma.markers.append(txt)

        # --- Workspace boundary (only when all 4 tags visible) ---
        if len(self._transforms) == len(self._tag_frames):
            ordered = self._order_ccw()
            pts_3d = [self._xyz(self._transforms[f]) for f in ordered]

            # Cyan LINE_STRIP (closed loop)
            line = Marker()
            line.header.stamp = now
            line.header.frame_id = self._cam
            line.ns = 'workspace_boundary'
            line.id = mid; mid += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.025
            line.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9)
            line.lifetime = lifetime
            for x, y, z in pts_3d + [pts_3d[0]]:
                line.points.append(Point(x=float(x), y=float(y), z=float(z)))
            ma.markers.append(line)

            # Corner-number labels along each edge mid-point
            for idx in range(len(pts_3d)):
                a = pts_3d[idx]
                b = pts_3d[(idx + 1) % len(pts_3d)]
                mid_pt = ((a[0]+b[0])/2, (a[1]+b[1])/2, (a[2]+b[2])/2)
                seg = Marker()
                seg.header.stamp = now
                seg.header.frame_id = self._cam
                seg.ns = 'workspace_edge_labels'
                seg.id = mid; mid += 1
                seg.type = Marker.TEXT_VIEW_FACING
                seg.action = Marker.ADD
                seg.pose.position.x = float(mid_pt[0])
                seg.pose.position.y = float(mid_pt[1])
                seg.pose.position.z = float(mid_pt[2]) - self._tag_size * 1.5
                seg.pose.orientation.w = 1.0
                seg.scale.z = 0.05
                seg.color = ColorRGBA(r=0.0, g=0.9, b=0.9, a=0.9)

                # Compute edge length in X-Y (floor plane)
                dx = b[0] - a[0]
                dy = b[1] - a[1]
                length = math.sqrt(dx*dx + dy*dy)
                seg.text = f'{length:.2f} m'
                seg.lifetime = lifetime
                ma.markers.append(seg)

            # Semi-transparent filled quad (TRIANGLE_LIST: two tris)
            fill = Marker()
            fill.header.stamp = now
            fill.header.frame_id = self._cam
            fill.ns = 'workspace_fill'
            fill.id = mid; mid += 1
            fill.type = Marker.TRIANGLE_LIST
            fill.action = Marker.ADD
            fill.scale.x = fill.scale.y = fill.scale.z = 1.0
            fill.color = ColorRGBA(r=0.0, g=0.85, b=0.85, a=0.12)
            fill.lifetime = lifetime
            for tri_idx in [(0, 1, 2), (0, 2, 3)]:
                for k in tri_idx:
                    px, py, pz = pts_3d[k]
                    fill.points.append(Point(x=float(px), y=float(py), z=float(pz)))
            ma.markers.append(fill)

            # workspace_origin sphere (centroid)
            cx = sum(p[0] for p in pts_3d) / 4
            cy_val = sum(p[1] for p in pts_3d) / 4
            cz = sum(p[2] for p in pts_3d) / 4
            orig = Marker()
            orig.header.stamp = now
            orig.header.frame_id = self._cam
            orig.ns = 'workspace_origin'
            orig.id = mid; mid += 1
            orig.type = Marker.SPHERE
            orig.action = Marker.ADD
            orig.pose.position.x = float(cx)
            orig.pose.position.y = float(cy_val)
            orig.pose.position.z = float(cz)
            orig.pose.orientation.w = 1.0
            orig.scale.x = orig.scale.y = orig.scale.z = 0.05
            orig.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            orig.lifetime = lifetime
            ma.markers.append(orig)

        self._pub_markers.publish(ma)

    # ------------------------------------------------------------------
    # Workspace origin TF
    # ------------------------------------------------------------------

    def _broadcast_workspace_origin(self):
        """
        Broadcast a dynamic TF: pi_camera → workspace_origin.

        The origin sits at the centroid of the 4 corner tags.  Its orientation
        is identical to the camera frame so that the local X-Y axes lie in the
        same plane as the tag positions (the floor plane when the camera is
        mounted overhead).  A future improvement can compute a proper rotation
        from the plane normal once floor-plane estimation is added.
        """
        pts = [self._xyz(self._transforms[f]) for f in self._tag_frames]
        cx = sum(p[0] for p in pts) / 4.0
        cy = sum(p[1] for p in pts) / 4.0
        cz = sum(p[2] for p in pts) / 4.0

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self._cam
        ts.child_frame_id = 'workspace_origin'
        ts.transform.translation.x = cx
        ts.transform.translation.y = cy
        ts.transform.translation.z = cz
        ts.transform.rotation.x = 0.0
        ts.transform.rotation.y = 0.0
        ts.transform.rotation.z = 0.0
        ts.transform.rotation.w = 1.0
        self._broadcaster.sendTransform(ts)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
