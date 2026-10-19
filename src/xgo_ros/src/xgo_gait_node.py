#!/usr/bin/env python3
"""XGO2 ROS2 node driving the custom first-principles gait (xgo_gait).

Wraps xgo_gait.GaitController over BLE:
  Subscribes : cmd_vel (Twist, m/s + rad/s), body_pose (Pose, optional)
  Publishes  : /joint_states, /imu/data, /battery

Parameters:
  bt_address   BLE MAC of the dog (default from xgo2_ros config)
  lib_path     path to the xgo_gait package parent dir
  gait         crawl | trot | pace        (default crawl)
  mode         joint | foot             (default joint: our IK -> 0x50)
  hz           control rate             (default 20)
  stand_z      foot z in leg frame mm   (default 95)
  calibrate    path to calibration.npz from calibrate_demo.py
"""

import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import BatteryState, Imu, JointState

DEFAULT_LIB = "/home/brian/EE5109/xgo_gait"
DEFAULT_ADDR = "38:18:2B:4F:E9:DA"


class XGoGaitNode(Node):

    def __init__(self):
        super().__init__('xgo_gait_node')

        self.declare_parameter('bt_address', DEFAULT_ADDR)
        self.declare_parameter('lib_path', DEFAULT_LIB)
        self.declare_parameter('gait', 'crawl')
        self.declare_parameter('mode', 'joint')
        self.declare_parameter('hz', 20.0)
        self.declare_parameter('stand_z', 95.0)
        self.declare_parameter('motor_speed', 200)
        self.declare_parameter('calibrate', '')
        self.declare_parameter('poll_hz', 8.0)

        p = lambda n: self.get_parameter(n).value
        lib = str(p('lib_path'))
        if lib not in sys.path:
            sys.path.insert(0, lib)
        try:
            from xgo_gait import (XGoLite, QuadKinematics, GaitController,
                                  Gait, crawl_params, trot_params,
                                  pace_params)
        except ImportError as exc:
            self.get_logger().fatal(
                f'cannot import xgo_gait from {lib}: {exc}')
            raise

        self._kin = QuadKinematics()
        cal = str(p('calibrate'))
        if cal:
            d = np.load(cal)
            self._kin.servo_sign = d['sign']
            self._kin.servo_offset_deg = d['offset']
            self.get_logger().info(f'loaded servo calibration {cal}')

        mk = {'crawl': crawl_params, 'trot': trot_params,
              'pace': pace_params}
        gp = mk[str(p('gait'))]()
        gp.stand_z = float(p('stand_z'))

        self._dog = XGoLite(str(p('bt_address')))
        self._dog.connect(timeout=60.0)
        self._dog.set_motor_speed(int(p('motor_speed')))
        self.get_logger().info(
            f'connected to {p("bt_address")}; '
            f'firmware {self._dog.read_firmware()}, '
            f'battery {self._dog.read_battery()}%')

        self._ctl = GaitController(self._dog, self._kin, Gait(gp),
                                   hz=float(p('hz')), mode=str(p('mode')))
        self._ctl.on_estop = lambda why: self.get_logger().error(
            f'e-stop: {why}')
        self._ctl.start(ramp_s=2.5)

        # pubs/subs
        self._joint_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        self._imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self._batt_pub = self.create_publisher(
            BatteryState, '/battery', 10)
        self.create_subscription(Twist, 'cmd_vel', self._vel_cb, 10)
        self.create_subscription(Pose, 'body_pose', self._pose_cb, 10)

        self.create_timer(1.0 / float(p('poll_hz')), self._poll)

    # ------------------------------------------------------------------
    def _vel_cb(self, msg: Twist):
        # ROS convention: m/s, rad/s -> controller: mm/s, rad/s
        self._ctl.set_velocity(msg.linear.x * 1000.0,
                               msg.linear.y * 1000.0,
                               msg.angular.z)

    def _pose_cb(self, msg: Pose):
        self._ctl.set_body_pose(
            translation_mm=[msg.position.x, msg.position.y,
                            msg.position.z],
            rpy_rad=[msg.orientation.x, msg.orientation.y,
                     msg.orientation.z])

    # ------------------------------------------------------------------
    def _poll(self):
        self._dog.request_state(motors=True, imu=True)
        self._publish_joint_state()
        self._publish_imu()

    def _publish_joint_state(self):
        ja = self._dog.motor_deg
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'lf_calf', 'lf_thigh', 'lf_hip',
            'rf_calf', 'rf_thigh', 'rf_hip',
            'rr_calf', 'rr_thigh', 'rr_hip',
            'lr_calf', 'lr_thigh', 'lr_hip']
        msg.position = np.radians(ja[:12]).tolist()
        self._joint_pub.publish(msg)

    def _publish_imu(self):
        roll, pitch, yaw = np.radians(self._dog.imu_deg)
        cr, sr = np.cos(roll / 2), np.sin(roll / 2)
        cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
        cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
        q = [sr * cp * cy - cr * sp * sy,
             cr * sp * cy + sr * cp * sy,
             cr * cp * sy - sr * sp * cy,
             cr * cp * cy + sr * sp * sy]
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        (msg.orientation.x, msg.orientation.y,
         msg.orientation.z, msg.orientation.w) = q
        self._imu_pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            self._ctl.stop()
            self._dog.reset()
            self._dog.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = XGoGaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
