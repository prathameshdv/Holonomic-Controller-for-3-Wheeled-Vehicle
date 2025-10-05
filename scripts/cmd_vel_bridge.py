#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.omni_pub = self.create_publisher(Float64MultiArray,
                                              '/omni_drive_controller/commands',
                                              10)

    def cmd_vel_callback(self, msg: Twist):
        cmd = Float64MultiArray()
        # Pass vx, vy, omega to omni
        cmd.data = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.omni_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
