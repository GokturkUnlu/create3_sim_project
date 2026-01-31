#!/usr/bin/env python3
"""
Twist to TwistStamped adapter node.
Converts geometry_msgs/msg/Twist messages to TwistStamped for Create 3 robot compatibility.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy


class TwistToTwistStampedAdapter(Node):
    def __init__(self):
        super().__init__('twist_adapter')
        
        # Parameter for use_sim_time
        self.declare_parameter('use_sim_time', True)
        
        # Subscribe to Nav2's Twist output
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )
        
        # Publish TwistStamped for the robot
        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel_stamped',
            10
        )
        
        self.get_logger().info('Twist to TwistStamped adapter started')
        self.get_logger().info('Subscribing to /cmd_vel (Twist), publishing to /cmd_vel_stamped (TwistStamped)')

    def twist_callback(self, msg: Twist):
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg
        self.publisher.publish(stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStampedAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
