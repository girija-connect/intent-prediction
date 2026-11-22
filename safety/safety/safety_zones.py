#!/usr/bin/env python3
"""
Two collision-avoidance zones around the robot, enforced on every velocity command.

    Nav2 / teleop ──► /cmd_vel ──► [ safety_zones ] ──► /cmd_vel_safe ──► Gazebo diff-drive
                                        ▲
                                     /scan (lidar)

    YELLOW zone (default 0.75 m): an obstacle is inside  -> Nav2 keeps planning, speed scaled to 50%
    RED    zone (default 0.35 m): an obstacle is inside  -> robot stops completely (zero velocity)

Zones are circles centred on base_link.  When the robot is driving, only obstacles in the
direction of travel count (so a wall beside you doesn't freeze you, and you can still back
away from something in front of you).  While rotating/standing still every direction counts.

Fail-safe: no lidar data for `scan_timeout` seconds -> stop.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .safety_core import zone_state

COLORS = {'red': (1.0, 0.05, 0.05), 'yellow': (1.0, 0.85, 0.0)}


class SafetyZones(Node):
    def __init__(self):
        super().__init__('safety_zones')
        d = self.declare_parameter
        d('red_radius', 0.35)        # m from base_link centre (robot body radius is ~0.22)
        d('yellow_radius', 0.75)     # m
        d('yellow_scale', 0.5)       # fraction of commanded speed allowed in the yellow zone
        d('hysteresis', 0.03)        # m, stops the state flickering at a zone edge
        d('min_points', 2)           # lidar hits needed inside a zone (rejects single noisy beams)
        d('self_radius', 0.20)       # ignore hits closer than this (the robot's own body)
        d('lidar_x', -0.064)         # lidar position in base_link (m, forward)
        d('lidar_y', 0.0)
        d('directional', True)       # only look where the robot is heading
        d('half_angle_deg', 100.0)   # width of that sector (+/- from the travel direction)
        d('scan_timeout', 0.5)       # s without a scan -> stop
        d('cmd_timeout', 0.5)        # s without a command -> publish zero
        d('rate', 30.0)
        d('cmd_in', '/cmd_vel')
        d('cmd_out', '/cmd_vel_safe')
        d('base_frame', 'base_link')
        self.g = lambda n: self.get_parameter(n).value

        self.pts = np.zeros((0, 2))
        self.scan_time = -1e9
        self.cmd, self.cmd_time = Twist(), -1e9
        self.state = 'clear'

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, self.g('cmd_in'), self.on_cmd, 10)
        self.pub_cmd = self.create_publisher(Twist, self.g('cmd_out'), 10)
        self.pub_state = self.create_publisher(String, '/safety_state', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/safety_zones', 10)
        self.create_timer(1.0 / self.g('rate'), self.step)
        self.get_logger().info(
            f"safety_zones: yellow<{self.g('yellow_radius')} m -> {self.g('yellow_scale'):.0%} speed, "
            f"red<{self.g('red_radius')} m -> stop.  {self.g('cmd_in')} -> {self.g('cmd_out')}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.cmd, self.cmd_time = msg, self.now()

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        # lidar frame -> base_link (lidar is mounted level, only translated)
        self.pts = np.column_stack((r[ok] * np.cos(a[ok]) + self.g('lidar_x'),
                                    r[ok] * np.sin(a[ok]) + self.g('lidar_y')))
        self.scan_time = self.now()

    def travel_direction(self, cmd):
        """Heading (rad, base_link frame) the robot is moving toward, or None if just rotating."""
        if abs(cmd.linear.x) < 0.01:
            return None
        return 0.0 if cmd.linear.x > 0 else math.pi

    def step(self):
        g = self.g
        now = self.now()
        cmd = self.cmd if now - self.cmd_time < g('cmd_timeout') else Twist()
        direction = self.travel_direction(cmd) if g('directional') else None

        if now - self.scan_time > g('scan_timeout'):
            state, nearest = 'red', float('nan')          # no lidar -> be safe
            reason = 'no /scan data'
        else:
            state, nearest = zone_state(
                self.pts, direction, g('red_radius'), g('yellow_radius'), g('self_radius'),
                math.radians(g('half_angle_deg')), int(g('min_points')),
                self.state, g('hysteresis'))
            reason = ''
        if state != self.state:
            self.get_logger().info(
                f"zone: {self.state.upper()} -> {state.upper()}  nearest obstacle "
                f"{nearest:.2f} m {reason}")
        self.state = state

        out = Twist()
        if state == 'yellow':
            s = g('yellow_scale')
            out.linear.x, out.angular.z = cmd.linear.x * s, cmd.angular.z * s
        elif state == 'clear':
            out = cmd
        # 'red' -> zero Twist: full stop
        self.pub_cmd.publish(out)
        self.pub_state.publish(String(data=f'{state} nearest={nearest:.2f}'))
        self.pub_markers.publish(self.markers(state))

    def markers(self, state):
        g = self.g
        arr = MarkerArray()
        for i, (name, radius, z) in enumerate((('yellow', g('yellow_radius'), 0.02),
                                               ('red', g('red_radius'), 0.03))):
            m = Marker()
            m.header.frame_id = g('base_frame')
            m.ns, m.id, m.type, m.action = 'zone_' + name, i, Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.z = 0.0, z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * radius
            m.scale.z = 0.01
            m.color.r, m.color.g, m.color.b = COLORS[name]
            active = (state == name) or (state == 'red' and name == 'yellow')
            m.color.a = 0.55 if active else 0.18          # lights up when triggered
            arr.markers.append(m)
        return arr


def main():
    rclpy.init()
    node = SafetyZones()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.pub_cmd.publish(Twist())          # leave the robot stopped
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
