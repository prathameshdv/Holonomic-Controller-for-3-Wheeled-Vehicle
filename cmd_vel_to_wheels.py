#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def clamp(x, a, b):
    return max(a, min(b, x))


class HolonomicController(Node):
    def __init__(self):
        super().__init__('holonomic_controller')

        # params
        self.declare_parameter('wheel_angles_deg', [0.0, 120.0, 240.0])
        self.declare_parameter('wheel_radius', 0.05)  # m
        self.declare_parameter('wheel_base', 0.25)    # m, distance center→wheel
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('max_wheel_speed', 20.0)
        self.declare_parameter('max_wheel_accel', 100.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.wheel_angles = [math.radians(a) for a in self.get_parameter('wheel_angles_deg').value]
        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.max_w = float(self.get_parameter('max_wheel_speed').value)
        self.max_a = float(self.get_parameter('max_wheel_accel').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)

        # pub & sub
        # NOTE: this publishes to the GroupVelocityController's command topic
        self.pub = self.create_publisher(Float64MultiArray, '/three_wheel_controller/commands', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # runtime vars
        self.desired_vx = self.desired_vy = self.desired_wz = 0.0
        self.current_omegas = [0.0, 0.0, 0.0]
        self.last_cmd_ts = self.get_clock().now()
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)
        self.get_logger().info("HolonomicController running...")

    def cmd_vel_cb(self, msg: Twist):
        self.desired_vx = msg.linear.x
        self.desired_vy = msg.linear.y
        self.desired_wz = msg.angular.z
        self.last_cmd_ts = self.get_clock().now()

    def compute_wheel_omegas(self, vx, vy, wz):
        omegas = []
        for theta in self.wheel_angles:
            # standard 3-wheel omni kinematic for wheel at angle theta
            v_w = -math.sin(theta) * vx + math.cos(theta) * vy + self.L * wz
            omegas.append(v_w / self.r)
        return omegas

    def timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9 or 1.0 / self.publish_rate

        # timeout safety
        if (now - self.last_cmd_ts).nanoseconds * 1e-9 > self.cmd_vel_timeout:
            vx = vy = wz = 0.0
        else:
            vx, vy, wz = self.desired_vx, self.desired_vy, self.desired_wz

        desired = self.compute_wheel_omegas(vx, vy, wz)

        # apply limits & smoothing
        out = Float64MultiArray()
        out.data = [0.0] * 3
        for i in range(3):
            d = clamp(desired[i], -self.max_w, self.max_w)
            delta = d - self.current_omegas[i]
            max_delta = self.max_a * dt
            delta = max(-max_delta, min(max_delta, delta))
            self.current_omegas[i] += delta
            out.data[i] = self.current_omegas[i]

        self.pub.publish(out)
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = HolonomicController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
