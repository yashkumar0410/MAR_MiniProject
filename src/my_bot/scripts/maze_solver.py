#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import math


class MazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Tuning parameters
        self.WALL_FOLLOW_DIST = 0.5   # desired distance to right wall (m)
        self.FRONT_CLEAR_DIST = 0.6   # obstacle threshold ahead (m)
        self.RIGHT_OPEN_DIST  = 0.7   # right is "open" if distance > this (m)

        self.LINEAR_SPEED     = 0.18  # forward speed (m/s)
        self.TURN_SPEED       = 0.55  # turning speed (rad/s)
        self.WALL_ALIGN_GAIN  = 1.2   # P-gain for right-wall following

        self.get_logger().info("Maze Solver started (right-hand rule)")

    def clean(self, values):
        """Remove inf/nan values from a range slice."""
        return [v for v in values if not math.isinf(v) and not math.isnan(v)]

    def sector_min(self, ranges, start, end):
        """Return min distance in a sector, defaulting to 10.0 if empty."""
        vals = self.clean(ranges[start:end])
        return min(vals) if vals else 10.0

    def scan_callback(self, msg):
        ranges = msg.ranges

        # Sector indices assume 360-ray LIDAR at 1-deg resolution,
        # index 0 = forward, increasing counter-clockwise.
        # Adjust if your LIDAR has a different ray count or zero-angle.
        front       = self.sector_min(ranges, 170, 190)   # dead ahead +/-10 deg
        front_right = self.sector_min(ranges, 120, 170)   # forward-right corner
        right       = self.sector_min(ranges,  80, 100)   # 90 deg right
        left        = self.sector_min(ranges, 260, 280)   # 90 deg left

        cmd = Twist()

        # Right-Hand Rule state machine
        if front < self.FRONT_CLEAR_DIST:
            # Obstacle ahead: turn left in place
            cmd.linear.x  = 0.0
            cmd.angular.z = self.TURN_SPEED
            state = "TURN_LEFT"

        elif right > self.RIGHT_OPEN_DIST:
            # Gap on the right: steer right to hug the wall
            cmd.linear.x  = self.LINEAR_SPEED * 0.6
            cmd.angular.z = -(self.TURN_SPEED * 0.8)
            state = "TURN_RIGHT"

        else:
            # Wall on the right: P-controller to maintain desired distance
            error = right - self.WALL_FOLLOW_DIST
            correction = self.WALL_ALIGN_GAIN * error
            cmd.linear.x  = self.LINEAR_SPEED
            cmd.angular.z = -correction
            state = "FOLLOW_WALL"

        self.pub.publish(cmd)

        self.get_logger().info(
            f"[{state}]  front={front:.2f}  right={right:.2f}"
            f"  fr={front_right:.2f}  left={left:.2f}"
            f"  lin={cmd.linear.x:.2f}  ang={cmd.angular.z:.2f}"
        )


def main():
    rclpy.init()
    node = MazeSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
