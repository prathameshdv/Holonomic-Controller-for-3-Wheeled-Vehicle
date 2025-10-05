#!/usr/bin/env python3
# File: three_wheel_odometry_node.py

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def invert_matrix(m):
    return np.linalg.inv(np.array(m))


class ThreeWheelOdometry(Node):
    def __init__(self):
        super().__init__('three_wheel_odometry')

        # Parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.25)
        self.declare_parameter('wheel_angles_deg', [0.0, 120.0, 240.0])
        self.declare_parameter('wheel_joints', ['wheel0_joint', 'wheel1_joint', 'wheel2_joint'])

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)
        self.wheel_angles = [math.radians(a) for a in self.get_parameter('wheel_angles_deg').value]
        self.wheel_joints = self.get_parameter('wheel_joints').value

        # Build kinematic matrix A (omegas = A * [vx, vy, wz])
        A = []
        for theta in self.wheel_angles:
            A.append([-math.sin(theta) / self.r,
                      math.cos(theta) / self.r,
                      self.L / self.r])
        self.A = np.array(A)
        self.A_inv = invert_matrix(self.A)

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None

        # ROS interfaces
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("ThreeWheelOdometry running...")

    def joint_cb(self, msg: JointState):
        # Extract wheel velocities from joint_states
        try:
            wheel_vels = []
            for j in self.wheel_joints:
                idx = msg.name.index(j)
                wheel_vels.append(msg.velocity[idx])
        except ValueError as e:
            self.get_logger().error(f"Wheel joint not found in JointState: {e}")
            return

        omegas = np.array(wheel_vels)

        # Compute robot velocity [vx, vy, wz]
        v = self.A_inv.dot(omegas)
        vx, vy, wz = v.tolist()

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Integrate pose
        dx = vx * math.cos(self.yaw) - vy * math.sin(self.yaw)
        dy = vx * math.sin(self.yaw) + vy * math.cos(self.yaw)
        self.x += dx * dt
        self.y += dy * dt
        self.yaw += wz * dt

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.pub_odom.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ThreeWheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
