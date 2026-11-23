#!/usr/bin/env python3
"""
Predict each human's intent from the tracker output and drive accordingly.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .intent_core import classify_intent, plan

COLORS = {'APPROACHING_HEAD_ON': (1.0, 0.1, 0.1), 'CROSSING_PATH': (1.0, 0.6, 0.0),
          'PASSING_BY': (0.3, 1.0, 0.3), 'MOVING_AWAY': (0.3, 0.8, 1.0),
          'STANDING': (0.8, 0.8, 0.8), 'UNKNOWN': (0.6, 0.6, 0.6)}


class IntentController(Node):
    def __init__(self):
        super().__init__('intent_controller')
        d = self.declare_parameter
        d('waypoints', [6.0, 0.0, 0.0, 0.0])   # x1,y1,x2,y2,... visited in a loop
        d('goal_tol', 0.3)
        d('v_max', 0.22)
        d('w_max', 1.2)
        d('social_radius', 1.2)
        d('safety_radius', 1.0)                # CPA distance counted as a conflict
        self.g = lambda n: self.get_parameter(n).value
        wp = self.g('waypoints')
        self.waypoints = [(wp[i], wp[i + 1]) for i in range(0, len(wp) - 1, 2)]
        self.wi = 0
        self.pose = None
        self.humans, self.hum_time = [], -1e9
        self.v_cmd = self.w_cmd = 0.0

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/human_tracks', self.on_tracks, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/intent_markers', 10)
        self.create_timer(0.1, self.control)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                     msg.twist.twist.linear.x)

    def on_tracks(self, msg):
        self.humans = json.loads(msg.data)['humans']
        self.hum_time = self.now()

    def control(self):
        if self.pose is None:
            return
        x, y, yaw, vb = self.pose
        goal = self.waypoints[self.wi]
        if math.hypot(goal[0] - x, goal[1] - y) < self.g('goal_tol'):
            self.wi = (self.wi + 1) % len(self.waypoints)
            goal = self.waypoints[self.wi]

        rpos = np.array([x, y])
        rvel = np.array([vb * math.cos(yaw), vb * math.sin(yaw)])
        plan_humans, markers, labels = [], MarkerArray(), []
        keep_right, v_cap = False, None

        if self.now() - self.hum_time < 1.0:
            for h in self.humans:
                pos = np.array([h['x'], h['y']])
                vel = np.array([h['vx'], h['vy']])
                moving = bool(h['valid'] and h['speed'] > 0.2)
                if h['valid']:
                    label, t_cpa, _ = classify_intent(
                        rpos, rvel, pos, vel, self.g('safety_radius'))
                else:
                    label, t_cpa = 'UNKNOWN', float('inf')
                plan_humans.append((pos, vel if moving else np.zeros(2), moving))
                labels.append((h, label, t_cpa))
                if label == 'APPROACHING_HEAD_ON' and t_cpa < 4.0:
                    keep_right = True                    # pass on the right
                if label == 'CROSSING_PATH' and t_cpa < 3.0:
                    v_cap = 0.5 * self.g('v_max')        # ease off, let them cross

        (v, w), _ = plan((x, y, yaw), goal, plan_humans,
                         prm=dict(v_max=self.g('v_max'), w_max=self.g('w_max'),
                                  social_r=self.g('social_radius')),
                         v_cap=v_cap, keep_right=keep_right)
        # accel limits (0.5 m/s^2, 4 rad/s^2) at 10 Hz
        self.v_cmd += float(np.clip(v - self.v_cmd, -0.05, 0.05))
        self.w_cmd += float(np.clip(w - self.w_cmd, -0.4, 0.4))
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = self.v_cmd, self.w_cmd
        self.pub_cmd.publish(cmd)

        for h, label, t_cpa in labels:
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns, m.id, m.type = 'intent', int(h['id']), Marker.TEXT_VIEW_FACING
            m.pose.position.x, m.pose.position.y, m.pose.position.z = h['x'], h['y'], 2.0
            m.scale.z = 0.25
            m.color.r, m.color.g, m.color.b = COLORS[label]
            m.color.a = 1.0
            m.text = f"#{h['id']} {label} {h['speed']:.1f}m/s"
            m.lifetime.nanosec = 300_000_000
            markers.markers.append(m)
            self.get_logger().info(
                f"human #{h['id']}: {label}  speed={h['speed']:.2f} m/s  "
                f"heading={math.degrees(h['heading']):.0f} deg  t_cpa={t_cpa:.1f}s",
                throttle_duration_sec=2.0)
        self.pub_markers.publish(markers)


def main():
    rclpy.init()
    node = IntentController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub_cmd.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
