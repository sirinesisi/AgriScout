#!/usr/bin/env python3
"""Pilote automatique AgriScout v2 - Version rapide et stable.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

N_LANES = 2
LANE_SPACING = 2.4
X_END = 7.5
SPEED = 0.15
K_HEAD, K_CROSS = 1.5, 1.2
W_MAX = 0.6
OBSTACLE_DIST = 0.25
SECTOR = math.radians(10)
ANGLE_TOL = math.radians(3)
EDGE_MARGIN = 0.3

# Correction : ajout de BACKOFF dans les états
LANE, TURN_A, SHIFT, TURN_B, AVOID, BACKOFF, DONE = 'LANE', 'TURN_A', 'SHIFT', 'TURN_B', 'AVOID', 'BACKOFF', 'DONE'

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def clamp(v, lim):
    return max(-lim, min(lim, v))

def lane_command(x, y, yaw, y_ref, direction, x_stop, speed=SPEED):
    heading = 0.0 if direction > 0 else math.pi
    remaining = direction * (x_stop - x)
    if remaining <= 0:
        return 0.0, 0.0, True
    v = max(0.05, min(speed, 0.6 * remaining))
    w = -K_HEAD * wrap(yaw - heading) - K_CROSS * direction * (y - y_ref)
    return v, clamp(w, W_MAX), False

def rotate_command(yaw, target):
    err = wrap(target - yaw)
    if abs(err) < ANGLE_TOL:
        return 0.0, True
    w = clamp(1.5 * err, W_MAX)
    if abs(w) < 0.15:
        w = math.copysign(0.15, err)
    return w, False

class AutoDrive(Node):
    def __init__(self):
        super().__init__('auto_drive')
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.status = self.create_publisher(String, '/mission/status', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.step)
        self.create_timer(1.0, self.announce)
        self.pose = None
        self.x0 = self.y0 = None
        self.lane = 0
        self.state = LANE
        self.blocked = False
        self.n_done = 0
        self.backoff_counter = 0
        self.get_logger().info('Pilote rapide pret.')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        if self.x0 is None:
            self.x0, self.y0 = p.x, p.y

    def on_scan(self, msg):
        near = [r for i, r in enumerate(msg.ranges)
                if math.isfinite(r) and r > msg.range_min
                and abs(wrap(msg.angle_min + i * msg.angle_increment)) < SECTOR]
        self.blocked = bool(near) and min(near) < OBSTACLE_DIST

    def send(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x, m.twist.angular.z = float(v), float(w)
        self.pub.publish(m)

    def set_state(self, s):
        self.state = s

    def step(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        direction = 1 if self.lane % 2 == 0 else -1
        y_ref = self.y0 + self.lane * LANE_SPACING
        v = w = 0.0

        if self.blocked and self.state == LANE:
            self.set_state(BACKOFF)
            self.backoff_counter = 0

        if self.state == LANE:
            x_stop = X_END if direction > 0 else self.x0 + EDGE_MARGIN
            v, w, fini = lane_command(x, y, yaw, y_ref, direction, x_stop)
            if fini:
                if self.lane + 1 >= N_LANES:
                    self.set_state(DONE)
                else:
                    self.set_state(TURN_A)

        elif self.state == BACKOFF:
            v = -0.10  # يرجع لوراء شوية
            w = 0.4    # يدور انحراف خفيف باش يخرج من العائق
            self.backoff_counter += 1
            if self.backoff_counter > 25:
                self.set_state(LANE)

        elif self.state == TURN_A:
            w, fini = rotate_command(yaw, math.pi / 2)
            if fini:
                self.set_state(SHIFT)

        elif self.state == SHIFT:
            y_next = self.y0 + (self.lane + 1) * LANE_SPACING
            if y >= y_next - 0.03:
                self.set_state(TURN_B)
            else:
                v = max(0.05, min(SPEED, 0.6 * (y_next - y)))
                w = clamp(K_HEAD * wrap(math.pi / 2 - yaw), W_MAX)

        elif self.state == TURN_B:
            next_dir = -direction
            target = 0.0 if next_dir > 0 else math.pi
            w, fini = rotate_command(yaw, target)
            if fini:
                self.lane += 1
                self.set_state(LANE)

        self.send(v, w)

    def announce(self):
        if self.state == DONE and self.n_done < 10:
            self.status.publish(String(data='done'))
            self.n_done += 1

def main(args=None):
    rclpy.init(args=args)
    node = AutoDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
