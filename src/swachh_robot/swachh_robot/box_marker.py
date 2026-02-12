#!/usr/bin/env python3
"""
Simple box marker publisher for RViz visualization.
Publishes a cube marker at the robot's position.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry


class BoxMarkerPublisher(Node):
    def __init__(self):
        super().__init__('box_marker_publisher')
        
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.get_logger().info('Box Marker Publisher started!')
        
    def odom_callback(self, msg):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position from odometry
        marker.pose = msg.pose.pose
        
        # Box size (0.3m x 0.3m x 0.2m)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.2
        
        # Green color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 0  # Forever
        
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = BoxMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
