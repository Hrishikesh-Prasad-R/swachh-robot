#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
import time

class AutonomousVacuum(Node):
    def __init__(self):
        super().__init__('autonomous_vacuum')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 
            'scan', 
            self.scan_callback, 
            10
        )
        
        # State variables
        self.state = 'FORWARD'  # States: FORWARD, TURNING
        self.obstacle_threshold = 0.5  # meters
        self.forward_speed = 0.2
        self.turn_speed = 0.8
        self.turn_start_time = None
        self.turn_duration = 0.0
        self.turn_direction = 1
        
        # Timer for control loop (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Store latest scan data
        self.latest_min_distance = float('inf')
        
        self.get_logger().info('Autonomous Vacuum Node Started')
        self.get_logger().info(f'Obstacle threshold: {self.obstacle_threshold}m')
        self.get_logger().info(f'Forward speed: {self.forward_speed} m/s')
    
    def scan_callback(self, msg):
        # Get minimum distance from laser scan
        # Filter out invalid readings (inf, nan)
        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        
        if valid_ranges:
            self.latest_min_distance = min(valid_ranges)
        else:
            self.get_logger().warn('No valid laser scan data')
    
    def control_loop(self):
        # Create velocity command
        twist = Twist()
        
        # State machine
        if self.state == 'FORWARD':
            if self.latest_min_distance < self.obstacle_threshold:
                # Obstacle detected - switch to turning
                self.state = 'TURNING'
                self.turn_start_time = time.time()
                self.turn_duration = random.uniform(1.0, 2.5)  # Random turn duration
                self.turn_direction = random.choice([-1, 1])  # Random turn direction
                self.get_logger().info(
                    f'Obstacle at {self.latest_min_distance:.2f}m - Turning for {self.turn_duration:.1f}s'
                )
            else:
                # Keep moving forward
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0
        
        elif self.state == 'TURNING':
            # Check if turn duration completed
            if time.time() - self.turn_start_time > self.turn_duration:
                self.state = 'FORWARD'
                self.get_logger().info('Turn complete - Moving forward')
            else:
                # Execute turn
                twist.linear.x = 0.0
                twist.angular.z = self.turn_direction * self.turn_speed
        
        # Publish velocity command
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousVacuum()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
