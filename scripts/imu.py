#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuRepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher')

        # Subscriber to raw IMU
        self.sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Publisher for fixed IMU
        self.pub = self.create_publisher(Imu, '/imu/data_cov', 10)

        self.get_logger().info("✅ IMU Republisher with covariance injection started...")

    def imu_callback(self, msg: Imu):
        # Inject non-zero covariances
        msg.orientation_covariance = [
            1e-3, 0.0, 0.0,
            0.0, 1e-3, 0.0,
            0.0, 0.0, 1e-3
        ]
        msg.angular_velocity_covariance = [
            1e-3, 0.0, 0.0,
            0.0, 1e-3, 0.0,
            0.0, 0.0, 1e-3
        ]
        msg.linear_acceleration_covariance = [
            1e-2, 0.0, 0.0,
            0.0, 1e-2, 0.0,
            0.0, 0.0, 1e-2
        ]

        # Republish
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
