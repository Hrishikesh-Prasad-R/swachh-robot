#!/usr/bin/env python3
"""
Environment and Robot Visualization — SLAM-Ready.
- Unified obstacle definitions (match Gazebo world exactly)
- Vision cone with bounding-box detection + enter/exit logging
- Async file logging (no terminal I/O at runtime)
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from swachh_robot.async_logger import get_logger, shutdown_logger
import math


# ========== SINGLE SOURCE OF TRUTH ==========
# These MUST match simple_room.world exactly.

ROOM1_CENTER = (0.0, 0.0)
ROOM2_CENTER = (9.0, 0.0)
ROOM_W = 6.0
ROOM_H = 6.0
WALL_THICKNESS = 0.1
WALL_HEIGHT = 0.5
CORRIDOR_WIDTH = 2.5
CORRIDOR_LENGTH = 3.0

# Obstacles: (name, x, y, size_x, size_y, size_z, r, g, b)
OBSTACLES = [
    # Room 1
    ('obs_1', 1.5,  1.5,  0.5, 0.5, 0.3,  0.8, 0.2, 0.2),
    ('obs_2', -1.5, 1.0,  0.4, 0.6, 0.25, 0.8, 0.5, 0.1),
    ('obs_3', 1.0,  -1.5, 0.6, 0.4, 0.35, 0.2, 0.7, 0.2),
    ('obs_4', -1.0, -1.0, 0.3, 0.3, 0.2,  0.7, 0.7, 0.2),
    # Room 2
    ('obs_5', 7.5,  1.0,  0.5, 0.5, 0.3,  0.8, 0.2, 0.2),
    ('obs_6', 10.5, -1.0, 0.4, 0.4, 0.25, 0.2, 0.2, 0.8),
    ('obs_7', 8.5,  -1.5, 0.6, 0.3, 0.3,  0.7, 0.2, 0.7),
]


class EnvironmentVisualizer(Node):
    """Publishes markers for walls, obstacles, robot, and vision cone."""

    def __init__(self):
        super().__init__('environment_visualizer')

        self.log = get_logger('env_viz')

        # Robot dimensions
        self.robot_width = 0.25
        self.robot_length = 0.3
        self.robot_height = 0.15
        self.wheel_radius = 0.04
        self.wheel_width = 0.02

        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Vision cone
        self.vision_range = 1.5
        self.vision_angle = 60  # degrees
        self.objects_in_vision = set()

        # Publishers
        self.wall_pub = self.create_publisher(MarkerArray, '/walls', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/obstacles', 10)
        self.robot_pub = self.create_publisher(MarkerArray, '/robot_visual', 10)
        self.vision_pub = self.create_publisher(Marker, '/robot_vision', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Timers
        self.create_timer(0.1, self.publish_robot)
        self.create_timer(0.1, self.publish_vision_cone)
        self.create_timer(0.2, self.check_vision_objects)
        self.create_timer(1.0, self.publish_environment)

        self.log.info(f'EnvironmentVisualizer started: 2 rooms, {len(OBSTACLES)} obstacles')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    # ==================== VISION CONE — BBOX DETECTION ====================

    def _point_in_cone(self, px, py):
        """Check if point is inside the vision cone. Returns (bool, dist)."""
        dx = px - self.robot_x
        dy = py - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > self.vision_range or dist < 0.01:
            return False, dist
        angle_to = math.atan2(dy, dx)
        rel = math.atan2(math.sin(angle_to - self.robot_yaw),
                         math.cos(angle_to - self.robot_yaw))
        half_fov = math.radians(self.vision_angle / 2.0)
        return abs(rel) <= half_fov, dist

    def _obstacle_in_cone(self, obs):
        """Check if any part of obstacle bbox is inside vision cone."""
        name, ox, oy, sx, sy = obs[0], obs[1], obs[2], obs[3], obs[4]
        hx, hy = sx / 2.0, sy / 2.0
        # Check center + 4 corners
        test_points = [
            (ox, oy),
            (ox - hx, oy - hy),
            (ox + hx, oy - hy),
            (ox - hx, oy + hy),
            (ox + hx, oy + hy),
        ]
        best_dist = float('inf')
        detected = False
        for px, py in test_points:
            in_cone, dist = self._point_in_cone(px, py)
            if in_cone:
                detected = True
                best_dist = min(best_dist, dist)
        return detected, best_dist

    def check_vision_objects(self):
        """Check which obstacles are visible — log enter/exit to file."""
        currently_visible = set()

        for obs in OBSTACLES:
            name = obs[0]
            detected, dist = self._obstacle_in_cone(obs)
            if detected:
                currently_visible.add(name)
                self.log.debug(f'VISION: {name} at {dist:.2f}m')

        entered = currently_visible - self.objects_in_vision
        for name in entered:
            self.log.warning(f'VISION_ENTER: {name}')

        exited = self.objects_in_vision - currently_visible
        for name in exited:
            self.log.warning(f'VISION_EXIT: {name}')

        self.objects_in_vision = currently_visible

    # ==================== WALL MARKERS ====================

    def _wall(self, mid, x, y, z, sx, sy, sz):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'walls'
        m.id = mid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = float(sx)
        m.scale.y = float(sy)
        m.scale.z = float(sz)
        m.color.r = 0.2
        m.color.g = 0.4
        m.color.b = 0.8
        m.color.a = 0.9
        return m

    def publish_environment(self):
        walls = MarkerArray()
        wh = 0.5   # wall height
        mz = 0.25  # wall center z

        # ===== HARDCODED FROM simple_room.world (no formulas) =====

        # Room 1 North:  pose(0, 3) size(6.2, 0.1, 0.5)
        walls.markers.append(self._wall(0,  0.0,    3.0,    mz, 6.2, 0.1, wh))
        # Room 1 South:  pose(0, -3) size(6.2, 0.1, 0.5)
        walls.markers.append(self._wall(1,  0.0,   -3.0,    mz, 6.2, 0.1, wh))
        # Room 1 West:   pose(-3, 0) size(0.1, 6.0, 0.5)
        walls.markers.append(self._wall(2, -3.0,    0.0,    mz, 0.1, 6.0, wh))
        # Room 1 East Top: pose(3, 2.125) size(0.1, 1.75, 0.5)
        walls.markers.append(self._wall(3,  3.0,    2.125,  mz, 0.1, 1.75, wh))
        # Room 1 East Bot: pose(3, -2.125) size(0.1, 1.75, 0.5)
        walls.markers.append(self._wall(4,  3.0,   -2.125,  mz, 0.1, 1.75, wh))

        # Corridor North:  pose(4.5, 1.25) size(3.0, 0.1, 0.5)
        walls.markers.append(self._wall(5,  4.5,    1.25,   mz, 3.0, 0.1, wh))
        # Corridor South:  pose(4.5, -1.25) size(3.0, 0.1, 0.5)
        walls.markers.append(self._wall(6,  4.5,   -1.25,   mz, 3.0, 0.1, wh))

        # Room 2 North:  pose(9, 3) size(6.2, 0.1, 0.5)
        walls.markers.append(self._wall(7,  9.0,    3.0,    mz, 6.2, 0.1, wh))
        # Room 2 South:  pose(9, -3) size(6.2, 0.1, 0.5)
        walls.markers.append(self._wall(8,  9.0,   -3.0,    mz, 6.2, 0.1, wh))
        # Room 2 East:   pose(12, 0) size(0.1, 6.0, 0.5)
        walls.markers.append(self._wall(9, 12.0,    0.0,    mz, 0.1, 6.0, wh))
        # Room 2 West Top: pose(6, 2.125) size(0.1, 1.75, 0.5)
        walls.markers.append(self._wall(10, 6.0,    2.125,  mz, 0.1, 1.75, wh))
        # Room 2 West Bot: pose(6, -2.125) size(0.1, 1.75, 0.5)
        walls.markers.append(self._wall(11, 6.0,   -2.125,  mz, 0.1, 1.75, wh))

        # Floors
        for fid, (fx, fy, fw, fh) in enumerate([
            (0.0, 0.0, 6.0, 6.0),     # Room 1
            (4.5, 0.0, 3.0, 2.5),     # Corridor
            (9.0, 0.0, 6.0, 6.0),     # Room 2
        ]):
            f = Marker()
            f.header.frame_id = 'odom'
            f.header.stamp = self.get_clock().now().to_msg()
            f.ns = 'walls'
            f.id = 20 + fid
            f.type = Marker.CUBE
            f.action = Marker.ADD
            f.pose.position.x = float(fx)
            f.pose.position.y = float(fy)
            f.pose.position.z = -0.01
            f.pose.orientation.w = 1.0
            f.scale.x = float(fw)
            f.scale.y = float(fh)
            f.scale.z = 0.02
            f.color.r = 0.4
            f.color.g = 0.4
            f.color.b = 0.4
            f.color.a = 0.5
            walls.markers.append(f)

        self.wall_pub.publish(walls)

        # Obstacles
        obs_arr = MarkerArray()
        for i, obs in enumerate(OBSTACLES):
            name, ox, oy, sx, sy, sz, cr, cg, cb = obs
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(ox)
            m.pose.position.y = float(oy)
            m.pose.position.z = float(sz) / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = float(sx)
            m.scale.y = float(sy)
            m.scale.z = float(sz)
            m.color.r = float(cr)
            m.color.g = float(cg)
            m.color.b = float(cb)
            m.color.a = 0.95
            obs_arr.markers.append(m)
        self.obstacle_pub.publish(obs_arr)

    # ==================== ROBOT MARKERS ====================

    def publish_robot(self):
        robot = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        body = Marker()
        body.header.frame_id = 'odom'
        body.header.stamp = stamp
        body.ns = 'robot'
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = self.robot_x
        body.pose.position.y = self.robot_y
        body.pose.position.z = self.robot_height / 2.0 + self.wheel_radius
        body.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
        body.pose.orientation.w = math.cos(self.robot_yaw / 2.0)
        body.scale.x = self.robot_length
        body.scale.y = self.robot_width
        body.scale.z = self.robot_height
        body.color.r = 0.2
        body.color.g = 0.8
        body.color.b = 0.2
        body.color.a = 1.0
        robot.markers.append(body)

        wheels = [
            (self.robot_length / 3, self.robot_width / 2 + self.wheel_width / 2),
            (self.robot_length / 3, -self.robot_width / 2 - self.wheel_width / 2),
            (-self.robot_length / 3, self.robot_width / 2 + self.wheel_width / 2),
            (-self.robot_length / 3, -self.robot_width / 2 - self.wheel_width / 2),
        ]
        cy = math.cos(self.robot_yaw)
        sy_val = math.sin(self.robot_yaw)
        for i, (dx, dy) in enumerate(wheels):
            w = Marker()
            w.header.frame_id = 'odom'
            w.header.stamp = stamp
            w.ns = 'robot'
            w.id = i + 1
            w.type = Marker.CYLINDER
            w.action = Marker.ADD
            w.pose.position.x = self.robot_x + dx * cy - dy * sy_val
            w.pose.position.y = self.robot_y + dx * sy_val + dy * cy
            w.pose.position.z = self.wheel_radius
            w.pose.orientation.x = math.sin(math.pi / 4)
            w.pose.orientation.z = math.sin(self.robot_yaw / 2.0) * math.cos(math.pi / 4)
            w.pose.orientation.w = math.cos(self.robot_yaw / 2.0) * math.cos(math.pi / 4)
            w.scale.x = self.wheel_radius * 2
            w.scale.y = self.wheel_radius * 2
            w.scale.z = self.wheel_width
            w.color.r = 0.1
            w.color.g = 0.1
            w.color.b = 0.1
            w.color.a = 1.0
            robot.markers.append(w)

        self.robot_pub.publish(robot)

    # ==================== VISION CONE ====================

    def publish_vision_cone(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'vision'
        m.id = 0
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD
        ha = math.radians(self.vision_angle / 2.0)
        p0 = Point(x=self.robot_x, y=self.robot_y, z=0.05)
        p1 = Point(
            x=self.robot_x + self.vision_range * math.cos(self.robot_yaw + ha),
            y=self.robot_y + self.vision_range * math.sin(self.robot_yaw + ha),
            z=0.05)
        p2 = Point(
            x=self.robot_x + self.vision_range * math.cos(self.robot_yaw - ha),
            y=self.robot_y + self.vision_range * math.sin(self.robot_yaw - ha),
            z=0.05)
        m.points = [p0, p1, p2]
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.color.r = 0.0
        m.color.g = 0.8
        m.color.b = 1.0
        m.color.a = 0.3
        self.vision_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        shutdown_logger()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
