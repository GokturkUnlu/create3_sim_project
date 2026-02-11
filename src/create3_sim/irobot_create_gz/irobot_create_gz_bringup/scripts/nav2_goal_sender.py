#!/usr/bin/env python3
"""
Nav2 Goal Sender - Send navigation goals to Nav2

Usage:
    python3 nav2_goal_sender.py --x 2.0 --y 1.0
    python3 nav2_goal_sender.py --x 2.0 --y 1.0 --yaw 1.57
"""

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        
        # Create action client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('Nav2 Goal Sender initialized')
    
    def send_goal(self, x: float, y: float, yaw: float = 0.0, frame_id: str = 'map'):
        """Send a navigation goal to Nav2."""
        
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available! Is Nav2 running?')
            return False
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion (only rotation around Z axis)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(
            f'Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad ({math.degrees(yaw):.1f}°) in frame "{frame_id}"'
        )
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True
    
    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by Nav2!')
            rclpy.shutdown()
            return
        
        self.get_logger().info('Goal accepted! Robot is navigating...')
        
        # Wait for result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)
    
    def _feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        
        self.get_logger().info(
            f'Current position: x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}'
        )
    
    def _result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation was aborted!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning('Navigation was canceled!')
        else:
            self.get_logger().warning(f'Navigation ended with status: {status}')
        
        rclpy.shutdown()


def main():
    # Parse custom arguments (before --ros-args)
    # Filter out ROS arguments for our own parsing
    custom_args = []
    ros_args_start = -1
    for i, arg in enumerate(sys.argv[1:], 1):
        if arg == '--ros-args':
            ros_args_start = i
            break
        custom_args.append(arg)
    
    # Parse our custom arguments
    x = 0.0
    y = 0.0
    yaw = 0.0
    frame = 'map'
    
    i = 0
    while i < len(custom_args):
        if custom_args[i] == '--x' and i + 1 < len(custom_args):
            x = float(custom_args[i + 1])
            i += 2
        elif custom_args[i] == '--y' and i + 1 < len(custom_args):
            y = float(custom_args[i + 1])
            i += 2
        elif custom_args[i] == '--yaw' and i + 1 < len(custom_args):
            yaw = float(custom_args[i + 1])
            i += 2
        elif custom_args[i] == '--frame' and i + 1 < len(custom_args):
            frame = custom_args[i + 1]
            i += 2
        elif custom_args[i] in ['-h', '--help']:
            print('Usage: nav2_goal_sender.py --x X --y Y [--yaw YAW] [--frame FRAME]')
            print('  --x X       Goal X position (meters) [required]')
            print('  --y Y       Goal Y position (meters) [required]')
            print('  --yaw YAW   Goal orientation in radians (default: 0.0)')
            print('  --frame     Reference frame (default: map)')
            return
        else:
            i += 1
    
    # Initialize ROS with all arguments (so ROS args like --ros-args work)
    rclpy.init(args=sys.argv)
    
    node = Nav2GoalSender()
    
    if node.send_goal(x, y, yaw, frame):
        rclpy.spin(node)
    else:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
