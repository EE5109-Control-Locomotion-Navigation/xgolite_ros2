#!/usr/bin/env python3
"""
Publish sensor_msgs/Image + CameraInfo from a V4L2 device via OpenCV.

Some virtual webcams (e.g. Iriun) expose frame intervals that usb_cam's
enumeration treats as unsupported, so usb_cam sees zero formats. OpenCV's
V4L2 backend often still captures those devices.
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


def _camera_info_from_yaml(path: Path, frame_id: str) -> CameraInfo:
    with path.open() as f:
        d = yaml.safe_load(f)
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = int(d['image_width'])
    msg.height = int(d['image_height'])
    msg.distortion_model = d['distortion_model']
    msg.k = [float(x) for x in d['camera_matrix']['data']]
    msg.d = [float(x) for x in d['distortion_coefficients']['data']]
    msg.r = [float(x) for x in d['rectification_matrix']['data']]
    msg.p = [float(x) for x in d['projection_matrix']['data']]
    return msg


def _open_video_capture(dev: str, logger) -> cv2.VideoCapture:
    """Try several OpenCV APIs; CAP_V4L2 often fails while the default backend works."""
    path = Path(dev)
    if not path.exists():
        raise RuntimeError(
            f'{dev} is not present in this environment. '
            'On the host run: v4l2-ctl --list-devices. '
            'In Docker, pass the device (see docker-compose.yml comments), e.g. '
            '- /dev/video2:/dev/video2 under services.ros2-xgolite.devices'
        )

    backends: list[tuple[int | None, str]] = []
    if hasattr(cv2, 'CAP_V4L2'):
        backends.append((cv2.CAP_V4L2, 'CAP_V4L2'))
    if hasattr(cv2, 'CAP_V4L'):
        backends.append((cv2.CAP_V4L, 'CAP_V4L'))
    backends.append((getattr(cv2, 'CAP_ANY', 0), 'CAP_ANY'))

    for api, name in backends:
        cap = cv2.VideoCapture(dev, api)
        if cap.isOpened():
            logger.info(f'Opened {dev} with OpenCV backend {name}')
            return cap
        cap.release()

    m = re.match(r'^/dev/video(\d+)$', dev)
    if m:
        idx = int(m.group(1))
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            logger.info(f'Opened V4L index {idx} (equivalent to {dev})')
            return cap
        cap.release()

    readable = os.access(dev, os.R_OK) if path.exists() else False
    raise RuntimeError(
        f'OpenCV could not open {dev} (exists={path.exists()}, '
        f'char_device={path.is_char_device()}, readable={readable}). '
        'Try another /dev/videoN or run outside Docker with direct device access.'
    )


class V4l2OpenCvCamNode(Node):
    def __init__(self) -> None:
        super().__init__('v4l2_opencv_cam')

        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('frame_id', 'pi_camera')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('camera_info_file', '')

        dev = self.get_parameter('video_device').value
        self._frame_id = self.get_parameter('frame_id').value
        w = int(self.get_parameter('image_width').value)
        h = int(self.get_parameter('image_height').value)
        fps = float(self.get_parameter('fps').value)
        ci_path = self.get_parameter('camera_info_file').value

        if not ci_path:
            self.get_logger().fatal('camera_info_file parameter is required')
            sys.exit(1)

        self._ci = _camera_info_from_yaml(Path(ci_path), self._frame_id)
        self._ci.width = w
        self._ci.height = h

        self._bridge = CvBridge()
        self._pub_img = self.create_publisher(Image, '/usb_cam/image_raw', 10)
        self._pub_ci = self.create_publisher(CameraInfo, '/usb_cam/camera_info', 10)

        try:
            self._cap = _open_video_capture(dev, self.get_logger())
        except RuntimeError as e:
            self.get_logger().fatal(str(e))
            sys.exit(1)

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self._cap.set(cv2.CAP_PROP_FPS, fps)

        aw = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if aw != w or ah != h:
            self.get_logger().warn(
                f'Requested {w}x{h} but device reports {aw}x{ah}; '
                'update launch image_width/height and camera_info to match.'
            )
            self._ci.width = aw
            self._ci.height = ah

        period = 1.0 / max(fps, 1.0)
        self._timer = self.create_timer(period, self._on_timer)
        self._failures = 0
        self.get_logger().info(
            f'Publishing /usb_cam/image_raw at {self._ci.width}x{self._ci.height} '
            f'frame_id={self._frame_id}'
        )

    def _on_timer(self) -> None:
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self._failures += 1
            if self._failures % 30 == 1:
                self.get_logger().warn('Frame grab failed (repeating)')
            return
        self._failures = 0

        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self._frame_id)
        try:
            img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8', header=header)
        except Exception as e:
            self.get_logger().error(f'cv2_to_imgmsg: {e}')
            return

        self._pub_img.publish(img_msg)
        self._ci.header.stamp = stamp
        self._pub_ci.publish(self._ci)

    def destroy_node(self):
        if hasattr(self, '_cap') and self._cap is not None:
            self._cap.release()
            self._cap = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = V4l2OpenCvCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
