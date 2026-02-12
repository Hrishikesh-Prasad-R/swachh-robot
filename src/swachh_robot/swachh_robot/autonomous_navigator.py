#!/usr/bin/env python3
"""
Autonomous Navigator for Swachh Robot

This node implements a simple autonomous navigation behavior for the Turtlebot3
that avoids obstacles and explores the environment.

States:
- FORWARD: Moving forward until obstacle detected
- TURNING: Rotating to avoid obstacle
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
import math


class AutonomousNavigator(Node):
    """Autonomous navigation node with obstacle avoidance."""

    # State constants
    STATE_FORWARD = 0
    STATE_TURNING = 1

    def __init__(self):
        super().__init__('autonomous_navigator')

        # Parameters (can be tuned)
        self.declare_parameter('obstacle_distance', 0.5)  # meters
        self.declare_parameter('forward_speed', 0.2)  # m/s
        self.declare_parameter('turn_speed', 0.5)  # rad/s
        self.declare_parameter('turn_duration', 2.0)  # seconds
        self.declare_parameter('scan_sectors', 5)  # number of sectors to analyze

        # Get parameters
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.turn_duration = self.get_parameter('turn_duration').value
        self.scan_sectors = self.get_parameter('scan_sectors').value

        # State variables
        self.current_state = self.STATE_FORWARD
        self.turn_direction = 1  # 1 = left, -1 = right
        self.turn_start_time = None
        self.min_distance = float('inf')
        self.sector_distances = []

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Autonomous Navigator started!')
        self.get_logger().info(f'Parameters: obstacle_dist={self.obstacle_distance}m, '
                               f'forward_speed={self.forward_speed}m/s, '
                               f'turn_speed={self.turn_speed}rad/s')

    def scan_callback(self, msg: LaserScan):
        """Process laser scan data."""
        ranges = msg.ranges
        num_ranges = len(ranges)

        if num_ranges == 0:
            return

        # Calculate minimum distance (ignoring inf values)
        valid_ranges = [r for r in ranges if not math.isinf(r) and not math.isnan(r)]
        if valid_ranges:
            self.min_distance = min(valid_ranges)
        else:
            self.min_distance = float('inf')

        # Divide scan into sectors for smarter navigation
        sector_size = num_ranges // self.scan_sectors
        self.sector_distances = []

        for i in range(self.scan_sectors):
            start_idx = i * sector_size
            end_idx = start_idx + sector_size
            sector_ranges = ranges[start_idx:end_idx]
            valid_sector = [r for r in sector_ranges if not math.isinf(r) and not math.isnan(r)]
            if valid_sector:
                self.sector_distances.append(min(valid_sector))
            else:
                self.sector_distances.append(float('inf'))

    def control_loop(self):
        """Main control loop - state machine."""
        cmd = Twist()
        current_time = self.get_clock().now()

        if self.current_state == self.STATE_FORWARD:
            # Check for obstacles
            if self.min_distance < self.obstacle_distance:
                # Obstacle detected - transition to turning
                self.current_state = self.STATE_TURNING
                self.turn_start_time = current_time

                # Choose turn direction based on sector distances
                # Find the sector with most open space
                if len(self.sector_distances) >= 2:
                    left_side = self.sector_distances[:len(self.sector_distances)//2]
                    right_side = self.sector_distances[len(self.sector_distances)//2:]

                    left_avg = sum(left_side) / len(left_side) if left_side else 0
                    right_avg = sum(right_side) / len(right_side) if right_side else 0

                    # Turn towards more open side with some randomness
                    if left_avg > right_avg:
                        self.turn_direction = 1  # Turn left
                    elif right_avg > left_avg:
                        self.turn_direction = -1  # Turn right
                    else:
                        # Equal or unable to determine - random choice
                        self.turn_direction = random.choice([1, -1])
                else:
                    self.turn_direction = random.choice([1, -1])

                # Add randomness to prevent repetitive patterns
                if random.random() < 0.2:  # 20% chance to flip direction
                    self.turn_direction *= -1

                self.get_logger().info(
                    f'Obstacle at {self.min_distance:.2f}m - turning '
                    f'{"left" if self.turn_direction > 0 else "right"}'
                )
            else:
                # No obstacle - move forward
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0

        elif self.current_state == self.STATE_TURNING:
            # Check if turn duration has elapsed
            elapsed = (current_time - self.turn_start_time).nanoseconds / 1e9

            if elapsed >= self.turn_duration:
                # Done turning - check if path is clear
                if self.min_distance >= self.obstacle_distance:
                    self.current_state = self.STATE_FORWARD
                    self.get_logger().info('Turn complete - resuming forward motion')
                else:
                    # Still blocked - continue turning a bit more
                    self.turn_start_time = current_time
                    # Possibly change direction
                    if random.random() < 0.3:  # 30% chance to try other direction
                        self.turn_direction *= -1
                        self.get_logger().info('Still blocked - trying opposite direction')
            else:
                # Still turning
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed * self.turn_direction

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)

    def destroy_node(self):
        """Clean shutdown - stop robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
