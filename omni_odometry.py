#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros


class OmniOdom(Node):
    def __init__(self):
        super().__init__('omni_odometry')

        # Robot parameters
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('L', 0.18)
        self.declare_parameter(
            'wheel_joints',
            ['left_wheel_joint', 'right_wheel_joint', 'back_wheel_joint']
        )

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('L').value
        self.wheel_joints = self.get_parameter('wheel_joints').value

        # Wheel orientation matrix for 120° spaced omni wheels
        self.angles = [0.0, 2 * math.pi / 3, 4 * math.pi / 3]
        self.W = np.array([[-math.sin(a), math.cos(a), self.L] for a in self.angles])
        self.Winv = np.linalg.pinv(self.W)

        # State
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.wheel_vels = [0.0, 0.0, 0.0]

        # Timestamps
        self.last_stamp = None

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.get_logger().info("✅ Omni odometry node started (TF + Odom publishing)")

    def joint_cb(self, msg: JointState):
        # Save current time from joint_states
        stamp = msg.header.stamp
        if self.last_stamp is None:
            self.last_stamp = stamp
            return

        # Compute dt in seconds
        dt = (stamp.sec - self.last_stamp.sec) + (stamp.nanosec - self.last_stamp.nanosec) * 1e-9
        if dt <= 0.0:
            return
        self.last_stamp = stamp

        # Extract wheel velocities
        for i, jname in enumerate(self.wheel_joints):
            if jname in msg.name:
                idx = msg.name.index(jname)
                if idx < len(msg.velocity):
                    self.wheel_vels[i] = msg.velocity[idx]

        # Compute body velocity
        omega = np.array(self.wheel_vels)
        v_body = self.r * self.Winv.dot(omega)  # [vx, vy, wz]
        vx_b, vy_b, wz = v_body

        # Integrate pose
        self.x += (vx_b * math.cos(self.yaw) - vy_b * math.sin(self.yaw)) * dt
        self.y += (vx_b * math.sin(self.yaw) + vy_b * math.cos(self.yaw)) * dt
        self.yaw += wz * dt

        # Quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)

        # ---- Odometry ----
        odom = Odometry()
        odom.header.stamp = stamp            # ✅ use joint state timestamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx_b
        odom.twist.twist.linear.y = vy_b
        odom.twist.twist.angular.z = wz

        # ✅ Inject realistic covariance so EKF accepts it
        odom.pose.covariance = [
            1e-3, 0.0,  0.0,   0.0,   0.0,   0.0,
            0.0,  1e-3, 0.0,   0.0,   0.0,   0.0,
            0.0,  0.0,  1e6,   0.0,   0.0,   0.0,
            0.0,  0.0,  0.0,   1e6,   0.0,   0.0,
            0.0,  0.0,  0.0,   0.0,   1e6,   0.0,
            0.0,  0.0,  0.0,   0.0,   0.0,   1e-2
        ]
        odom.twist.covariance = [
            1e-3, 0.0,  0.0,   0.0,   0.0,   0.0,
            0.0,  1e-3, 0.0,   0.0,   0.0,   0.0,
            0.0,  0.0,  1e6,   0.0,   0.0,   0.0,
            0.0,  0.0,  0.0,   1e6,   0.0,   0.0,
            0.0,  0.0,  0.0,   0.0,   1e6,   0.0,
            0.0,  0.0,  0.0,   0.0,   0.0,   1e-2
        ]


        self.odom_pub.publish(odom)

        # ---- TF ----
        t = TransformStamped()
        t.header.stamp = stamp               # ✅ same timestamp as odom
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OmniOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
