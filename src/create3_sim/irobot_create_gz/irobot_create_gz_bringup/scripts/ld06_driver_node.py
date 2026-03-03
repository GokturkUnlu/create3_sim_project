#!/usr/bin/env python3
"""
PuduBot LD06 Lidar ROS 2 Driver Node

Reads raw lidar data from the PuduBot via ADB and publishes LaserScan.

Supports two input modes:
  1. ADB subprocess (default): Runs 'adb exec-out cat /dev/ttyS2' internally
  2. TCP: Connects to a TCP server (for use with external forwarder)

PuduBot LD06 Protocol (reverse-engineered):
  Frame: CE FA <n_points> 00 <start_angle_LE16> <hdr2(2)> <n_points × 3 bytes>
  - 8-byte header, then N × 3 bytes (distance_mm LE + confidence)
  - 100 frames per 360° rotation, ~10 Hz
"""

import os
import sys
import socket
import struct
import math
import time
import threading
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


FRAME_HEADER_0 = 0xCE
FRAME_HEADER_1 = 0xFA
FRAME_HEADER_SIZE = 8


class LD06DriverNode(Node):
    def __init__(self):
        super().__init__('ld06_driver')

        # Parameters
        self.declare_parameter('source', 'adb')       # 'adb' or 'tcp'
        self.declare_parameter('adb_serial', '')       # ADB device serial (empty = default)
        self.declare_parameter('lidar_dev', '/dev/ttyS2')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 4001)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('angle_offset', 0.0)
        self.declare_parameter('scan_frequency', 10.0)
        self.declare_parameter('min_confidence', 10)
        self.declare_parameter('reconnect_delay', 2.0)

        self.source = self.get_parameter('source').value
        self.adb_serial = self.get_parameter('adb_serial').value
        self.lidar_dev = self.get_parameter('lidar_dev').value
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_offset = self.get_parameter('angle_offset').value
        self.scan_frequency = self.get_parameter('scan_frequency').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.reconnect_delay = self.get_parameter('reconnect_delay').value

        # Scan accumulator
        self.scan_points = []
        self.last_start_angle = -1.0
        self.scan_start_time = None

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Statistics
        self.frame_count = 0
        self.scan_count = 0
        self.bad_frames = 0
        self.bytes_received = 0

        # Data reading thread
        self.running = True
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

        # Stats timer
        self.create_timer(10.0, self._print_stats)

        self.get_logger().info(
            f'PuduBot LD06 driver started: source={self.source}, '
            f'frame={self.frame_id}, range=[{self.range_min}, {self.range_max}]m'
        )

    # ── Data Input ────────────────────────────────────────────────────────

    def _recv_loop(self):
        """Background thread: read data from selected source."""
        buffer = bytearray()

        while self.running:
            try:
                if self.source == 'adb':
                    self._recv_adb(buffer)
                elif self.source == 'tcp':
                    self._recv_tcp(buffer)
                else:
                    self.get_logger().error(f'Unknown source: {self.source}')
                    break
            except Exception as e:
                self.get_logger().error(f'Source error: {e}')

            if self.running:
                self.get_logger().warn(
                    f'Data stream ended. Reconnecting in {self.reconnect_delay}s...'
                )
                time.sleep(self.reconnect_delay)

    def _recv_adb(self, buffer):
        """Read lidar data via 'adb exec-out cat <device>'."""
        cmd = ['adb']
        if self.adb_serial:
            cmd.extend(['-s', self.adb_serial])
        cmd.extend(['exec-out', f'cat {self.lidar_dev}'])

        self.get_logger().info(f'Starting ADB stream: {" ".join(cmd)}')

        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0
        )

        try:
            while self.running and proc.poll() is None:
                data = proc.stdout.read(4096)
                if not data:
                    break
                self.bytes_received += len(data)
                buffer.extend(data)
                self._process_buffer(buffer)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()

        stderr_out = proc.stderr.read().decode(errors='replace').strip()
        if stderr_out:
            self.get_logger().warn(f'ADB stderr: {stderr_out}')

    def _recv_tcp(self, buffer):
        """Read lidar data from a TCP server."""
        self.get_logger().info(f'Connecting to {self.host}:{self.port}...')
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((self.host, self.port))
        sock.settimeout(1.0)
        self.get_logger().info('Connected to TCP lidar stream!')

        try:
            while self.running:
                try:
                    data = sock.recv(4096)
                    if not data:
                        break
                    self.bytes_received += len(data)
                    buffer.extend(data)
                    self._process_buffer(buffer)
                except socket.timeout:
                    continue
        finally:
            sock.close()

    # ── Frame Parsing ─────────────────────────────────────────────────────

    def _process_buffer(self, buffer):
        """Extract and parse complete frames from the buffer."""
        while len(buffer) >= FRAME_HEADER_SIZE:
            # Find frame header CE FA
            idx = -1
            for i in range(len(buffer) - 1):
                if buffer[i] == FRAME_HEADER_0 and buffer[i + 1] == FRAME_HEADER_1:
                    idx = i
                    break

            if idx < 0:
                buffer.clear()
                return

            if idx > 0:
                del buffer[:idx]

            if len(buffer) < FRAME_HEADER_SIZE:
                return

            n_points = buffer[2]

            if n_points < 10 or n_points > 200:
                self.bad_frames += 1
                del buffer[:2]
                continue

            frame_size = FRAME_HEADER_SIZE + n_points * 3

            if len(buffer) < frame_size:
                return

            frame = bytes(buffer[:frame_size])
            del buffer[:frame_size]

            self._parse_frame(frame, n_points)

    def _parse_frame(self, frame, n_points):
        """Parse a single Pudu LD06 frame."""
        self.frame_count += 1

        start_angle_raw = struct.unpack_from('<H', frame, 4)[0]
        start_angle_deg = start_angle_raw / 100.0

        # start_angle_deg is a sector ID (0.0, 3.6, 7.2, ..., 32.4)
        # Actual angular position = sector_id × 10 (so 0°, 36°, 72°, ..., 324°)
        # Each of the 10 sectors covers 36° with ~92 points
        actual_start_deg = start_angle_deg * 10.0
        angle_span_deg = 36.0
        angle_step_deg = angle_span_deg / n_points

        # Detect full rotation: sector ID drops (e.g. 32.4° → 0°)
        if self.last_start_angle >= 0:
            if start_angle_deg < self.last_start_angle:
                self._publish_scan()

        self.last_start_angle = start_angle_deg

        if self.scan_start_time is None:
            self.scan_start_time = self.get_clock().now()

        for i in range(n_points):
            offset = FRAME_HEADER_SIZE + i * 3
            if offset + 2 >= len(frame):
                break

            distance_mm = struct.unpack_from('<H', frame, offset)[0]
            confidence = frame[offset + 2]

            angle_deg = actual_start_deg + i * angle_step_deg
            if angle_deg >= 360.0:
                angle_deg -= 360.0

            # Pudu: 0° forward, CW positive → ROS: 0 rad forward, CCW positive
            angle_rad = -math.radians(angle_deg) + self.angle_offset

            while angle_rad > math.pi:
                angle_rad -= 2 * math.pi
            while angle_rad < -math.pi:
                angle_rad += 2 * math.pi

            distance_m = distance_mm / 1000.0
            self.scan_points.append((angle_rad, distance_m, confidence))

    def _publish_scan(self):
        """Assemble and publish a LaserScan from accumulated data."""
        if len(self.scan_points) < 50:
            self.scan_points = []
            self.scan_start_time = None
            return

        now = self.get_clock().now()
        scan_duration = 1.0 / self.scan_frequency

        self.scan_points.sort(key=lambda p: p[0])

        n = len(self.scan_points)
        angle_min = self.scan_points[0][0]
        angle_max = self.scan_points[-1][0]
        angle_increment = (angle_max - angle_min) / (n - 1) if n > 1 else 0.01

        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.time_increment = scan_duration / n
        msg.scan_time = scan_duration
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        msg.ranges = []
        msg.intensities = []
        for (angle, dist, conf) in self.scan_points:
            if conf < self.min_confidence or dist < self.range_min or dist > self.range_max:
                msg.ranges.append(float('inf'))
            else:
                msg.ranges.append(dist)
            msg.intensities.append(float(conf))

        self.scan_pub.publish(msg)
        self.scan_count += 1

        self.scan_points = []
        self.scan_start_time = None

    def _print_stats(self):
        """Log periodic statistics."""
        kb = self.bytes_received / 1024
        self.get_logger().info(
            f'Stats: {self.scan_count} scans, {self.frame_count} frames, '
            f'{self.bad_frames} bad, {kb:.1f} KB received'
        )

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LD06DriverNode()
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
