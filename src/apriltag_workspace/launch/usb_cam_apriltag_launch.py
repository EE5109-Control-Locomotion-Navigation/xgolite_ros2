"""
V4L2 webcam (e.g. Iriun phone camera) + apriltag_ros for local development.

Iriun installs a virtual camera on Linux; find it with:
  sudo apt install v4l-utils && v4l2-ctl --list-devices
Often /dev/video0 is the built-in cam and Iriun is /dev/video2 (varies by machine).

Many virtual cameras report no *discrete* V4L2 frame intervals, so usb_cam enumerates
zero formats and fails for every pixel_format. In that case use the default driver:

  camera_driver:=opencv

Usage:
  ros2 launch apriltag_workspace usb_cam_apriltag_launch.py video_device:=/dev/video2
  ros2 launch apriltag_workspace usb_cam_apriltag_launch.py camera_driver:=usb_cam pixel_format:=yuyv2rgb

Then in another terminal:
  ros2 launch apriltag_workspace workspace_launch.py

Requires: ros-humble-apriltag-ros, ros-humble-cv-bridge, python3-opencv
Optional: ros-humble-usb-cam (only for camera_driver:=usb_cam)
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _generate(context, *args, **kwargs):
    pkg = get_package_share_directory('apriltag_workspace')
    video_device = LaunchConfiguration('video_device').perform(context)
    image_width = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))
    framerate = float(LaunchConfiguration('framerate').perform(context))
    camera_driver = LaunchConfiguration('camera_driver').perform(context)

    cam_info = Path(pkg) / 'config' / 'camera_info_iriun_approx.yaml'
    apriltag_cfg = Path(pkg) / 'config' / 'apriltag_workspace_tags.yaml'
    camera_info_url = f'file://{cam_info.resolve()}'
    camera_info_file = str(cam_info.resolve())

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[str(apriltag_cfg)],
        remappings=[
            ('image_rect', '/usb_cam/image_raw'),
            ('camera_info', '/usb_cam/camera_info'),
        ],
    )

    if camera_driver == 'usb_cam':
        pixel_format = LaunchConfiguration('pixel_format').perform(context)
        usb_cam = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': video_device,
                'framerate': framerate,
                'io_method': LaunchConfiguration('io_method').perform(context),
                'frame_id': 'pi_camera',
                'pixel_format': pixel_format,
                'image_width': image_width,
                'image_height': image_height,
                'camera_name': 'pi_camera',
                'camera_info_url': camera_info_url,
            }],
        )
        return [usb_cam, apriltag]

    if camera_driver == 'opencv':
        opencv_cam = Node(
            package='apriltag_workspace',
            executable='v4l2_opencv_cam',
            name='v4l2_opencv_cam',
            output='screen',
            parameters=[{
                'video_device': video_device,
                'frame_id': 'pi_camera',
                'image_width': image_width,
                'image_height': image_height,
                'fps': framerate,
                'camera_info_file': camera_info_file,
            }],
        )
        return [opencv_cam, apriltag]

    raise RuntimeError(f'Unknown camera_driver: {camera_driver!r} (use opencv or usb_cam)')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_driver',
            default_value='opencv',
            description=(
                'opencv: OpenCV V4L2 (works around broken usb_cam enum on some virtual cams, e.g. Iriun). '
                'usb_cam: ros-humble-usb-cam when the device lists discrete frame intervals.'
            ),
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='V4L2 device (Iriun virtual camera)',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='1280',
            description='Must match stream and camera_info_iriun_approx.yaml width',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='720',
            description='Must match stream and camera_info_iriun_approx.yaml height',
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='yuyv2rgb',
            description='usb_cam only: yuyv2rgb, mjpeg2rgb, uyvy2rgb, m4202rgb, raw_mjpeg, yuyv, uyvy, ...',
        ),
        DeclareLaunchArgument(
            'io_method',
            default_value='mmap',
            description='usb_cam only: mmap, read, or userptr',
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='30.0',
            description='Capture framerate',
        ),
        OpaqueFunction(function=_generate),
    ])
