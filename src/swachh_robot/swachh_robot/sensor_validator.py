#!/usr/bin/env python3
"""
Sensor Integrity Validator + Coordinate Frame Checker.

Monitors:
- /scan rate, data quality, orientation, units
- TF chain: odom → base_footprint → base_link → base_scan
- Timestamp consistency

All output goes to async file logger. No terminal I/O.
"""

import rclpy
from rclpy.node import Node
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


class SensorValidator(Node):
    """Continuous sensor and frame validation node."""

    def __init__(self):
        super().__init__('sensor_validator')
        self.log = get_logger('sensor_validator')

        # Scan timing
        self.scan_times = collections.deque(maxlen=50)
        self.scan_count = 0
        self.inf_count = 0
        self.nan_count = 0
        self.total_readings = 0

        # Odom timing
        self.odom_times = collections.deque(maxlen=50)
        self.odom_count = 0

        # Expected rates
        self.expected_scan_hz = 5.0
        self.expected_odom_hz = 30.0

        # First scan metadata
        self.scan_meta_logged = False

        # TF
        if HAS_TF2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.log.info('TF2 available, will validate frame chain')
        else:
            self.tf_buffer = None
            self.log.warning('TF2 not available, skipping frame validation')

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Periodic validation reports
        self.create_timer(5.0, self.report)
        self.create_timer(10.0, self.check_tf)

        self.log.info('SensorValidator started')

    def scan_cb(self, msg):
        now = time.monotonic()
        self.scan_times.append(now)
        self.scan_count += 1

        # Data quality
        readings = msg.ranges
        n = len(readings)
        self.total_readings += n
        infs = sum(1 for r in readings if math.isinf(r))
        nans = sum(1 for r in readings if math.isnan(r))
        self.inf_count += infs
        self.nan_count += nans

        # Log metadata on first scan
        if not self.scan_meta_logged:
            self.log.info(
                f'SCAN_META: angle_min={msg.angle_min:.4f} angle_max={msg.angle_max:.4f} '
                f'angle_incr={msg.angle_increment:.6f} range_min={msg.range_min:.3f} '
                f'range_max={msg.range_max:.3f} readings={n} frame={msg.header.frame_id}'
            )
            # Unit check: range_max should be in meters (typically 3.5 for burger)
            if msg.range_max > 100:
                self.log.error(f'UNIT_ERROR: range_max={msg.range_max:.1f} — likely centimeters, expected meters!')
            else:
                self.log.info(f'UNIT_CHECK: range_max={msg.range_max:.1f}m — OK (meters)')

            # Orientation check (burger: -pi to pi, 360 readings)
            expected_span = 2 * math.pi
            actual_span = abs(msg.angle_max - msg.angle_min)
            if abs(actual_span - expected_span) > 0.1:
                self.log.warning(f'ORIENT_WARN: scan span={actual_span:.3f} expected~{expected_span:.3f}')
            else:
                self.log.info(f'ORIENT_CHECK: scan spans {math.degrees(actual_span):.0f}° — OK')

            self.scan_meta_logged = True

    def odom_cb(self, msg):
        now = time.monotonic()
        self.odom_times.append(now)
        self.odom_count += 1

    def _calc_rate(self, times):
        """Calculate Hz from timestamp deque."""
        if len(times) < 2:
            return 0.0
        dt = times[-1] - times[0]
        if dt <= 0:
            return 0.0
        return (len(times) - 1) / dt

    def report(self):
        """Periodic validation report to log file."""
        scan_hz = self._calc_rate(self.scan_times)
        odom_hz = self._calc_rate(self.odom_times)

        # Scan rate check
        if self.scan_count > 0:
            deviation = abs(scan_hz - self.expected_scan_hz) / self.expected_scan_hz * 100
            level = 'INFO' if deviation < 10 else 'WARNING'
            self.log.log(
                __import__('logging').getLevelName(level),
                f'SCAN_RATE: {scan_hz:.1f}Hz (expected {self.expected_scan_hz}Hz, deviation {deviation:.0f}%)'
            )
        else:
            self.log.error('SCAN_RATE: NO SCANS RECEIVED')

        # Odom rate check
        if self.odom_count > 0:
            self.log.info(f'ODOM_RATE: {odom_hz:.1f}Hz (count={self.odom_count})')
        else:
            self.log.error('ODOM_RATE: NO ODOM RECEIVED')

        # Data quality
        if self.total_readings > 0:
            inf_pct = self.inf_count / self.total_readings * 100
            nan_pct = self.nan_count / self.total_readings * 100
            if nan_pct > 0:
                self.log.error(f'DATA_QUALITY: {nan_pct:.1f}% NaN readings!')
            if inf_pct > 50:
                self.log.warning(f'DATA_QUALITY: {inf_pct:.1f}% inf (walls far away?)')
            else:
                self.log.info(f'DATA_QUALITY: {inf_pct:.1f}% inf, {nan_pct:.1f}% NaN — OK')

        # Frame rate stability (check for gaps)
        if len(self.scan_times) >= 3:
            intervals = [self.scan_times[i+1] - self.scan_times[i]
                        for i in range(len(self.scan_times)-1)]
            max_gap = max(intervals)
            min_gap = min(intervals)
            if max_gap > 2 * min_gap and min_gap > 0:
                self.log.warning(f'TIMING_JITTER: scan interval range [{min_gap:.3f}s, {max_gap:.3f}s]')
            else:
                self.log.info(f'TIMING_STABLE: scan intervals [{min_gap:.3f}s, {max_gap:.3f}s]')

    def check_tf(self):
        """Validate TF frame chain."""
        if not self.tf_buffer:
            return

        frames_to_check = [
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
            ('base_link', 'base_scan'),
        ]

        all_ok = True
        for parent, child in frames_to_check:
            try:
                t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                age = (self.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds / 1e9
                if age > 1.0:
                    self.log.warning(f'TF_STALE: {parent}→{child} age={age:.1f}s')
                    all_ok = False
                else:
                    tx = t.transform.translation
                    self.log.debug(
                        f'TF_OK: {parent}→{child} '
                        f'xyz=({tx.x:.3f},{tx.y:.3f},{tx.z:.3f}) age={age:.3f}s'
                    )
            except Exception as e:
                self.log.error(f'TF_MISSING: {parent}→{child}: {e}')
                all_ok = False

        if all_ok:
            self.log.info('TF_CHAIN: complete and fresh')


def main(args=None):
    rclpy.init(args=args)
    node = SensorValidator()
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
