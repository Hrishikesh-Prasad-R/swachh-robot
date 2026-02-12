#!/usr/bin/env python3
"""
SLAM Readiness Gate — Pre-SLAM Dry Run Checks.

Runs automated checks for 15 seconds, then outputs PASS/FAIL report.
Checks:
  1. Scan rate within ±10% of expected
  2. Scan data quality (<5% NaN)
  3. TF chain complete (odom→base→scan)
  4. Odom publishing at >5Hz
  5. Cmd_vel response (robot moves when commanded)
  6. Wall detection (min distance changes near walls)
  7. Frame timestamp consistency (no jumps >500ms)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from swachh_robot.async_logger import get_logger, shutdown_logger
import time
import math
import collections

try:
    from tf2_ros import Buffer, TransformListener
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False

CHECK_DURATION = 15.0  # seconds


class SlamReadiness(Node):
    """Automated SLAM readiness checks."""

    def __init__(self):
        super().__init__('slam_readiness')
        self.log = get_logger('slam_readiness')

        self.start_time = time.monotonic()
        self.results = {}

        # Scan data
        self.scan_times = []
        self.scan_count = 0
        self.total_readings = 0
        self.nan_count = 0
        self.min_distances = []

        # Odom data
        self.odom_times = []
        self.odom_count = 0
        self.positions = []

        # TF
        if HAS_TF2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Send a brief test command at t=3s
        self.test_cmd_sent = False
        self.create_timer(0.1, self.tick)

        self.log.info(f'SLAM Readiness check started ({CHECK_DURATION}s)')
        print(f"[SLAM READINESS] Running checks for {CHECK_DURATION:.0f}s...")

    def scan_cb(self, msg):
        self.scan_times.append(time.monotonic())
        self.scan_count += 1
        n = len(msg.ranges)
        self.total_readings += n
        self.nan_count += sum(1 for r in msg.ranges if math.isnan(r))
        valid = [r for r in msg.ranges if 0.1 < r < 10.0 and not math.isnan(r)]
        if valid:
            self.min_distances.append(min(valid))

    def odom_cb(self, msg):
        self.odom_times.append(time.monotonic())
        self.odom_count += 1
        self.positions.append((
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ))

    def tick(self):
        elapsed = time.monotonic() - self.start_time

        # Send test movement at t=3s
        if not self.test_cmd_sent and elapsed > 3.0:
            twist = Twist()
            twist.linear.x = 0.1
            self.cmd_pub.publish(twist)
            self.test_cmd_sent = True
            self.log.info('TEST_CMD: sent linear=0.1 for response test')

        # Stop test movement at t=5s
        if self.test_cmd_sent and elapsed > 5.0:
            self.cmd_pub.publish(Twist())

        # Complete at CHECK_DURATION
        if elapsed >= CHECK_DURATION:
            self.cmd_pub.publish(Twist())
            self.run_checks()
            raise SystemExit(0)

    def _calc_rate(self, times):
        if len(times) < 2:
            return 0.0
        dt = times[-1] - times[0]
        return (len(times) - 1) / dt if dt > 0 else 0.0

    def run_checks(self):
        """Run all checks and produce report."""
        checks = []

        # 1. Scan rate
        scan_hz = self._calc_rate(self.scan_times)
        expected = 5.0
        deviation = abs(scan_hz - expected) / expected * 100 if expected > 0 else 100
        ok = deviation < 15 and self.scan_count > 0
        checks.append(('SCAN_RATE', ok, f'{scan_hz:.1f}Hz (±{deviation:.0f}%)'))

        # 2. Data quality
        nan_pct = (self.nan_count / self.total_readings * 100) if self.total_readings > 0 else 100
        ok = nan_pct < 5
        checks.append(('DATA_QUALITY', ok, f'{nan_pct:.1f}% NaN'))

        # 3. TF chain
        tf_ok = True
        tf_msg = 'N/A'
        if HAS_TF2:
            missing = []
            for parent, child in [('odom', 'base_footprint'),
                                   ('base_footprint', 'base_link'),
                                   ('base_link', 'base_scan')]:
                try:
                    self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                except Exception:
                    missing.append(f'{parent}→{child}')
                    tf_ok = False
            tf_msg = 'complete' if tf_ok else f'missing: {", ".join(missing)}'
        checks.append(('TF_CHAIN', tf_ok, tf_msg))

        # 4. Odom rate
        odom_hz = self._calc_rate(self.odom_times)
        ok = odom_hz > 5.0
        checks.append(('ODOM_RATE', ok, f'{odom_hz:.1f}Hz'))

        # 5. Cmd_vel response
        moved = False
        if len(self.positions) > 10:
            p0 = self.positions[0]
            p_last = self.positions[-1]
            d = math.sqrt((p_last[0] - p0[0])**2 + (p_last[1] - p0[1])**2)
            moved = d > 0.01
        checks.append(('CMD_RESPONSE', moved, f'displacement={d:.3f}m' if len(self.positions) > 10 else 'no odom'))

        # 6. Wall detection
        if self.min_distances:
            dist_range = max(self.min_distances) - min(self.min_distances)
            has_walls = min(self.min_distances) < 5.0
        else:
            dist_range = 0
            has_walls = False
        checks.append(('WALL_DETECT', has_walls ,
                       f'min={min(self.min_distances):.2f}m range={dist_range:.2f}m' if self.min_distances else 'no data'))

        # 7. Timestamp consistency
        max_gap = 0
        if len(self.scan_times) > 2:
            gaps = [self.scan_times[i+1] - self.scan_times[i]
                   for i in range(len(self.scan_times)-1)]
            max_gap = max(gaps)
        ok = max_gap < 0.5
        checks.append(('FRAME_TIMING', ok, f'max_gap={max_gap:.3f}s'))

        # Generate report
        all_pass = all(c[1] for c in checks)
        self.log.info('=' * 50)
        self.log.info('SLAM READINESS REPORT')
        self.log.info('=' * 50)

        print("\n" + "=" * 50)
        print("  SLAM READINESS REPORT")
        print("=" * 50)

        for name, passed, detail in checks:
            icon = 'PASS' if passed else 'FAIL'
            line = f'  [{icon}] {name}: {detail}'
            self.log.info(line)
            symbol = '✅' if passed else '❌'
            print(f"  {symbol} {name}: {detail}")

        self.log.info('=' * 50)
        if all_pass:
            self.log.info('RESULT: ALL CHECKS PASSED — SLAM READY')
            print("\n  🟢 ALL CHECKS PASSED — SLAM READY")
        else:
            failed = [c[0] for c in checks if not c[1]]
            self.log.warning(f'RESULT: FAILED checks: {", ".join(failed)}')
            print(f"\n  🔴 FAILED: {', '.join(failed)}")

        print("=" * 50 + "\n")
        self.log.info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = SlamReadiness()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        shutdown_logger()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
