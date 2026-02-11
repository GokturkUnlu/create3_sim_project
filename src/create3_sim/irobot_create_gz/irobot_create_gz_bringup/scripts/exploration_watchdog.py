#!/usr/bin/env python3
"""
Exploration Watchdog - Fast Escalation with Resume & Backtrack

Key feature: After every recovery, publishes True on /explore/resume
to restart explore_lite's frontier search (it stops when its goal is
canceled by the watchdog).

Recovery escalation:
  1. Clear costmaps
  2. Spin 180° + clear
  3. Backup 0.3m + clear
  4+ Backtrack to nearest visited point 2-4m away

Recovery counter resets only when robot escapes 1.5m+ from stuck origin.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose, Spin, BackUp
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
import math
import time
import random


class ExplorationWatchdog(Node):
    def __init__(self):
        super().__init__('exploration_watchdog')

        self.declare_parameter('stuck_threshold', 25.0)
        self.declare_parameter('movement_threshold', 0.15)
        self.declare_parameter('escape_threshold', 1.5)
        self.declare_parameter('check_interval', 5.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')

        self.stuck_threshold = self.get_parameter('stuck_threshold').value
        self.movement_threshold = self.get_parameter('movement_threshold').value
        self.escape_threshold = self.get_parameter('escape_threshold').value
        self.check_interval = self.get_parameter('check_interval').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        # State
        self.last_position = None
        self.last_significant_move_time = time.time()
        self.recovering = False
        self.recovery_attempts = 0
        self.stuck_origin = None
        self.visited_positions = []
        self.failed_escape_points = []

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher to resume explore_lite after recovery
        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)

        # Nav2 action clients
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.backup_client = ActionClient(self, BackUp, '/backup')

        # Costmap service clients
        self.clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )
        self.clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
        )

        self.check_timer = self.create_timer(self.check_interval, self.check_stuck)
        self.position_tracker = self.create_timer(5.0, self.track_position)

        self.get_logger().info(
            f'Watchdog with resume: stuck={self.stuck_threshold}s, '
            f'escape={self.escape_threshold}m'
        )

    # ── helpers ──────────────────────────────────────────────
    def get_pos(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            return t.transform.translation
        except Exception:
            return None

    def d(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def track_position(self):
        p = self.get_pos()
        if p is None:
            return
        if not self.visited_positions or \
           self.d(p.x, p.y, *self.visited_positions[-1]) > 0.3:
            self.visited_positions.append((p.x, p.y))
            if len(self.visited_positions) > 100:
                self.visited_positions.pop(0)

    def resume_exploration(self):
        """Publish True on /explore/resume to restart explore_lite."""
        msg = Bool()
        msg.data = True
        self.resume_pub.publish(msg)
        self.get_logger().info('Published resume to explore_lite')

    # ── stuck detection ─────────────────────────────────────
    def check_stuck(self):
        if self.recovering:
            return
        p = self.get_pos()
        if p is None:
            return

        if self.last_position is None:
            self.last_position = p
            self.last_significant_move_time = time.time()
            return

        dist = self.d(p.x, p.y, self.last_position.x, self.last_position.y)
        if dist > self.movement_threshold:
            self.last_significant_move_time = time.time()
            self.last_position = p
            if self.stuck_origin:
                esc = self.d(p.x, p.y, *self.stuck_origin)
                if esc > self.escape_threshold:
                    self.get_logger().info(
                        f'✓ Escaped {esc:.1f}m from stuck origin - counter reset')
                    self.recovery_attempts = 0
                    self.stuck_origin = None
        else:
            dt = time.time() - self.last_significant_move_time
            if dt > self.stuck_threshold:
                self.get_logger().warn(
                    f'Stuck {dt:.0f}s at ({p.x:.2f},{p.y:.2f}) – '
                    f'attempt #{self.recovery_attempts + 1}')
                self.initiate_recovery(p)

    # ── recovery ────────────────────────────────────────────
    def initiate_recovery(self, pos):
        self.recovering = True
        self.recovery_attempts += 1

        if self.stuck_origin is None:
            self.stuck_origin = (pos.x, pos.y)

        try:
            self.cancel_goal()

            if self.recovery_attempts == 1:
                self.get_logger().info('▸ L1: Clear costmaps')
                self.clear_costmaps()

            elif self.recovery_attempts == 2:
                self.get_logger().info('▸ L2: Spin 180°')
                self.clear_costmaps()
                self.try_spin(math.pi)

            elif self.recovery_attempts == 3:
                self.get_logger().info('▸ L3: Backup 0.3m')
                self.clear_costmaps()
                self.try_backup()

            else:
                self.get_logger().info(f'▸ L4: Backtrack escape (attempt {self.recovery_attempts})')
                self.clear_costmaps()
                self.backtrack_escape(pos)

        except Exception as e:
            self.get_logger().error(f'Recovery error: {e}')

        wait = 20.0 if self.recovery_attempts >= 4 else 10.0
        self.create_timer(wait, self._finish)

    def backtrack_escape(self, pos):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('Nav2 not ready')
            return

        candidates = []
        for (px, py) in self.visited_positions:
            dist = self.d(px, py, pos.x, pos.y)
            if dist < 2.0 or dist > 6.0:
                continue
            too_close = False
            for (fx, fy) in self.failed_escape_points:
                if self.d(px, py, fx, fy) < 1.0:
                    too_close = True
                    break
            if too_close:
                continue
            candidates.append((px, py, dist))

        if candidates:
            candidates.sort(key=lambda c: c[2])
            target = (candidates[0][0], candidates[0][1])
        else:
            dist_to_origin = self.d(pos.x, pos.y, 0.0, 0.0)
            if dist_to_origin > 1.5:
                target = (pos.x * 0.3, pos.y * 0.3)
            else:
                a = random.uniform(0, 2 * math.pi)
                target = (2.0 * math.cos(a), 2.0 * math.sin(a))

        self.failed_escape_points.append(target)
        if len(self.failed_escape_points) > 20:
            self.failed_escape_points.pop(0)

        self.get_logger().info(
            f'  Backtrack: ({pos.x:.1f},{pos.y:.1f}) → ({target[0]:.1f},{target[1]:.1f})')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target[0]
        goal.pose.pose.position.y = target[1]
        goal.pose.pose.orientation.w = 1.0
        self.nav_client.send_goal_async(goal)

    # ── Nav2 actions ────────────────────────────────────────
    def try_spin(self, angle):
        try:
            if not self.spin_client.server_is_ready():
                return
            g = Spin.Goal()
            g.target_yaw = angle
            g.time_allowance.sec = 20
            self.spin_client.send_goal_async(g)
        except Exception:
            pass

    def try_backup(self):
        try:
            if not self.backup_client.server_is_ready():
                return
            g = BackUp.Goal()
            g.target.x = -0.3
            g.time_allowance.sec = 10
            g.speed = 0.05
            self.backup_client.send_goal_async(g)
        except Exception:
            pass

    def cancel_goal(self):
        try:
            c = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
            if not c.wait_for_service(timeout_sec=2.0):
                return
            r = CancelGoal.Request()
            r.goal_info.stamp.sec = 0
            r.goal_info.stamp.nanosec = 0
            c.call_async(r)
        except Exception:
            pass

    def clear_costmaps(self):
        try:
            if self.clear_global.service_is_ready():
                self.clear_global.call_async(ClearEntireCostmap.Request())
            if self.clear_local.service_is_ready():
                self.clear_local.call_async(ClearEntireCostmap.Request())
        except Exception:
            pass

    def _finish(self):
        if self.recovering:
            self.get_logger().info(f'Recovery done (total: {self.recovery_attempts})')
            self.last_significant_move_time = time.time()
            self.last_position = self.get_pos()
            self.recovering = False
            # CRITICAL: Resume explore_lite so it starts finding new frontiers
            self.resume_exploration()


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
