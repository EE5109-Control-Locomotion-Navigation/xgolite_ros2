#!/usr/bin/env python3
"""
IMU to Odometry Node
Converts orientation (roll, pitch, yaw) from the robot IMU into an Odometry message.
Subscribes: /imu/data (sensor_msgs/Imu)
Publishes: /odom_orientation (nav_msgs/Odometry) — orientation only, position at origin, twist zeroed
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class ImuToOdomNode(Node):
    """Publishes odom message with pose orientation from IMU, zero position and twist."""

    def __init__(self):
        super().__init__('imu_to_odom_node')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom_orientation')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.create_subscription(Imu, imu_topic, self._imu_cb, 10)

        self.get_logger().info(
            f'imu_to_odom: {imu_topic} -> {odom_topic}'
        )

    def _imu_cb(self, msg: Imu):
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame_id
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = msg.orientation
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ImuToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
