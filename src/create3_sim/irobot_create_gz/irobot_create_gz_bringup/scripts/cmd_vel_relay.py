#!/usr/bin/env python3
"""
Simple topic relay: forwards Twist messages from /cmd_vel to /cmd_vel_unstamped.
This bridges Nav2's Twist output to Create 3's motion_control input.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscribe to Nav2's collision_monitor output (Twist)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )
        
        # Publish to motion_control's unstamped input
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_unstamped',
            10
        )
        
        self.get_logger().info('CmdVel relay: /cmd_vel (Nav2) -> /cmd_vel_unstamped (robot)')

    def callback(self, msg: Twist):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
