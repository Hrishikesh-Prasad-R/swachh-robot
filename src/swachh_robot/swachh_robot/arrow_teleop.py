#!/usr/bin/env python3
"""
Arrow Key Teleop — SLAM-Ready.
- Speed levels 1–9 (press number key)
- Proportional deceleration: robot smoothly slows and stops exactly at walls
- 120° collision scan (wide safety) + 60° vision cone (for logging)
- No invisible walls — collision boundary = robot body
- 150ms key hold eliminates turning stutter
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from swachh_robot.async_logger import get_logger, shutdown_logger
import sys
import os
import termios
import tty
import select
import time
import math
import logging
import collections
import statistics

# Speed table: level → (linear m/s, angular rad/s)
SPEED_LEVELS = {
    1: (0.05, 0.10),
    2: (0.10, 0.20),
    3: (0.15, 0.30),
    4: (0.20, 0.45),
    5: (0.30, 0.60),   # default
    6: (0.40, 0.75),
    7: (0.50, 0.90),
    8: (0.65, 1.20),
    9: (0.80, 1.50),
}

# Collision geometry
ROBOT_RADIUS = 0.105        # TurtleBot3 burger physical radius (m)
SLOW_ZONE = 0.25            # Start decelerating this far from contact (m)
COLLISION_HALF_ANGLE = 60    # ±60° = 120° total for collision scan
VISION_HALF_ANGLE = 30       # ±30° = 60° total for vision cone distance

# Input
KEY_HOLD_SEC = 0.15          # Bridge OS key-repeat gaps


class ArrowTeleop(Node):
    """Teleop with proportional collision and vision-cone distance."""

    def __init__(self):
        super().__init__('arrow_teleop')
        self.log = get_logger('teleop')

        # Speed
        self.speed_level = 5
        self.linear_speed, self.angular_speed = SPEED_LEVELS[5]

        # Motion state
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_key_time = 0.0
        self.key_count = 0

        # Scan distances (separate for collision vs vision)
        self.collision_dist = float('inf')    # min dist in 120° front sector
        self.vision_dist = float('inf')       # min dist in 60° vision cone
        self.collision_history = collections.deque(maxlen=3)
        self.vision_history = collections.deque(maxlen=3)

        # Scan metadata
        self.scan_initialized = False

        # Terminal
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)

        # Publisher / subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_timer(1.0, self.status_log)

        self.log.info('ArrowTeleop started, speed_level=5')

    # ==================== BANNER ====================

    def print_banner(self):
        sys.stdout.write("\r\n" + "=" * 60 + "\r\n")
        sys.stdout.write("   ARROW KEY TELEOP — SLAM-READY\r\n")
        sys.stdout.write("=" * 60 + "\r\n")
        sys.stdout.write("   UP/DOWN  = Forward/Backward\r\n")
        sys.stdout.write("   LEFT/RIGHT = Turn\r\n")
        sys.stdout.write("   1-9      = Speed level (1=slow, 9=fast)\r\n")
        sys.stdout.write("   SPACE    = Emergency stop\r\n")
        sys.stdout.write("   Q        = Quit\r\n")
        sys.stdout.write(f"   Speed: Level {self.speed_level} "
                         f"(lin={self.linear_speed:.2f}, ang={self.angular_speed:.2f})\r\n")
        sys.stdout.write("=" * 60 + "\r\n\r\n")
        sys.stdout.flush()

    # ==================== SCAN — DUAL SECTOR ====================

    def scan_callback(self, msg):
        """Extract min distance from two sectors: 120° collision + 60° vision."""
        n = len(msg.ranges)
        if n == 0:
            return

        if not self.scan_initialized:
            self.scan_initialized = True
            self.log.info(
                f'SCAN_INIT: n={n} angle_min={msg.angle_min:.3f} '
                f'incr={msg.angle_increment:.5f} range_max={msg.range_max:.1f}m'
            )

        collision_readings = []
        vision_readings = []

        collision_half_rad = math.radians(COLLISION_HALF_ANGLE)
        vision_half_rad = math.radians(VISION_HALF_ANGLE)

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r < 0.02:
                continue

            # Angle of this ray (normalized to [-π, π])
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            abs_angle = abs(angle)

            if r < msg.range_max and abs_angle <= collision_half_rad:
                collision_readings.append(r)

            if r < msg.range_max and abs_angle <= vision_half_rad:
                vision_readings.append(r)

        # Collision: 120° sector
        if collision_readings:
            self.collision_history.append(min(collision_readings))
            self.collision_dist = statistics.median(self.collision_history)
        else:
            self.collision_dist = float('inf')

        # Vision: 60° cone
        if vision_readings:
            self.vision_history.append(min(vision_readings))
            self.vision_dist = statistics.median(self.vision_history)
        else:
            self.vision_dist = float('inf')

        self.log.debug(
            f'scan: collision_dist={self.collision_dist:.3f}m '
            f'vision_dist={self.vision_dist:.3f}m'
        )

    # ==================== COLLISION — PROPORTIONAL ====================

    def get_speed_factor(self):
        """Proportional speed scaling based on proximity.
        Returns a factor in [0.0, 1.0]:
        - 1.0 = full speed (far from obstacles)
        - 0.0 = hard stop (at or past robot boundary)
        The robot body touches at collision_dist == ROBOT_RADIUS."""
        gap = self.collision_dist - ROBOT_RADIUS
        if gap <= 0.0:
            return 0.0   # At or past boundary — hard stop
        if gap >= SLOW_ZONE:
            return 1.0   # Far enough — full speed
        # Linear ramp between boundary and slow zone edge
        return gap / SLOW_ZONE

    def get_effective_forward_speed(self):
        """Forward speed after collision scaling. Exact-boundary stop."""
        factor = self.get_speed_factor()
        effective = self.linear_speed * factor
        # Minimum meaningful speed (below this, just stop)
        if effective < 0.01:
            return 0.0
        return effective

    # ==================== STATUS ====================

    def status_log(self):
        status = "MOVING" if (self.current_linear != 0 or self.current_angular != 0) else "STOPPED"
        direction = ""
        if self.current_linear > 0:
            direction = "FWD"
        elif self.current_linear < 0:
            direction = "REV"
        if self.current_angular > 0:
            direction += "+L"
        elif self.current_angular < 0:
            direction += "+R"
        if not direction:
            direction = "IDLE"

        factor = self.get_speed_factor()
        gap = self.collision_dist - ROBOT_RADIUS
        obs_info = (f"wall_gap={gap:.2f}m factor={factor:.0%}" 
                    if self.collision_dist < 5 else "clear")
        self.log.info(
            f'STATUS: {status} dir={direction} {obs_info} '
            f'speed_level={self.speed_level} keys={self.key_count}'
        )

    # ==================== INPUT ====================

    def read_key(self):
        rlist, _, _ = select.select([self.fd], [], [], 0.01)
        if rlist:
            ch = os.read(self.fd, 1)
            if not ch:
                return None
            if ch == b'\x1b':
                more, _, _ = select.select([self.fd], [], [], 0.05)
                if more:
                    ch += os.read(self.fd, 2)
            return ch.decode('utf-8', errors='ignore')
        return None

    # ==================== KEY PROCESSING ====================

    def process_key(self, key):
        """Process key → Twist with proportional collision scaling."""
        twist = Twist()
        action = None

        # Speed level
        if key in '123456789':
            level = int(key)
            self.speed_level = level
            self.linear_speed, self.angular_speed = SPEED_LEVELS[level]
            self.log.info(f'SPEED: level={level} lin={self.linear_speed:.2f} ang={self.angular_speed:.2f}')
            if self.current_linear > 0:
                twist.linear.x = self.get_effective_forward_speed()
            elif self.current_linear < 0:
                twist.linear.x = -self.linear_speed
            if self.current_angular > 0:
                twist.angular.z = self.angular_speed
            elif self.current_angular < 0:
                twist.angular.z = -self.angular_speed
            return twist, f"SPEED {level}"

        if key == '\x1b[A':  # UP — proportional collision
            effective = self.get_effective_forward_speed()
            if effective <= 0.0:
                gap = self.collision_dist - ROBOT_RADIUS
                action = f"BLOCKED gap={gap:.3f}m"
                twist.linear.x = 0.0
            else:
                twist.linear.x = effective
                if self.get_speed_factor() < 1.0:
                    action = f"FORWARD(slow {self.get_speed_factor():.0%})"
                else:
                    action = "FORWARD"
        elif key == '\x1b[B':  # DOWN
            twist.linear.x = -self.linear_speed
            action = "BACKWARD"
        elif key == '\x1b[C':  # RIGHT
            twist.angular.z = -self.angular_speed
            action = "TURN_RIGHT"
        elif key == '\x1b[D':  # LEFT
            twist.angular.z = self.angular_speed
            action = "TURN_LEFT"
        elif key == ' ':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            action = "ESTOP"
        elif key in ('q', 'Q'):
            return None, "QUIT"
        else:
            return twist, None

        self.current_linear = twist.linear.x
        self.current_angular = twist.angular.z
        self.key_count += 1

        if action:
            blocked = twist.linear.x == 0.0 and 'BLOCKED' in action
            self.log.log(
                logging.WARNING if blocked else logging.INFO,
                f'INPUT: {action} vel=({twist.linear.x:.2f}, {twist.angular.z:.2f}) '
                f'speed_level={self.speed_level}'
            )

        return twist, action

    # ==================== MAIN LOOP ====================

    def run(self):
        tty.setraw(self.fd)
        self.print_banner()
        self.log.info('Control loop started')

        try:
            while rclpy.ok():
                key = self.read_key()
                now = time.monotonic()

                if key:
                    self.last_key_time = now
                    twist, action = self.process_key(key)

                    if action == "QUIT":
                        self.cmd_pub.publish(Twist())
                        self.log.info('QUIT requested')
                        break

                    if twist:
                        self.cmd_pub.publish(twist)
                else:
                    # Key-hold: keep command for KEY_HOLD_SEC to bridge repeat gaps
                    if self.current_linear != 0 or self.current_angular != 0:
                        if (now - self.last_key_time) > KEY_HOLD_SEC:
                            self.current_linear = 0.0
                            self.current_angular = 0.0
                            self.cmd_pub.publish(Twist())

                # CONTINUOUS collision enforcement (runs every cycle, key or not)
                if self.current_linear > 0:
                    effective = self.get_effective_forward_speed()
                    if effective < self.current_linear:
                        # Scale down or stop
                        self.current_linear = effective
                        twist = Twist()
                        twist.linear.x = effective
                        twist.angular.z = self.current_angular
                        self.cmd_pub.publish(twist)
                        if effective == 0.0:
                            self.log.warning(
                                f'COLLISION_STOP: gap={self.collision_dist - ROBOT_RADIUS:.3f}m'
                            )

                rclpy.spin_once(self, timeout_sec=0.005)

        except KeyboardInterrupt:
            self.log.info('Interrupted')
        finally:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.old_term)
            self.cmd_pub.publish(Twist())
            self.log.info('Robot stopped, terminal restored')
            print("\n[INFO] Robot stopped. Goodbye!")


def main(args=None):
    rclpy.init(args=args)
    node = ArrowTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        shutdown_logger()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
