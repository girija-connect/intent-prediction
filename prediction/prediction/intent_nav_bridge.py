#!/usr/bin/env python3
"""
Bridge between human-intent prediction and Nav2.  It never publishes /cmd_vel.

    /human_tracks ──► intent (CPA) ──┬─► /human_predicted_grid       (OccupancyGrid)  ─► Nav2 local costmap
                                     ├─► /speed_limit                (nav2 SpeedLimit) ─► Nav2 controller
                                     ├─► /human_intents              (JSON, for logging / other nodes)
                                     └─► /intent_markers             (RViz labels + predicted paths)

Nav2 keeps driving to the goal you; this node makes it slow down for people who are 
about to cross / walk into it, and makes it steer around where they WILL be.
"""
import json
import math

import numpy as np
import rclpy
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .intent_core import classify_intent, predicted_discs, rasterize_discs, speed_scale

COLORS = {'APPROACHING_HEAD_ON': (1.0, 0.1, 0.1), 'CROSSING_PATH': (1.0, 0.6, 0.0),
          'PASSING_BY': (0.3, 1.0, 0.3), 'MOVING_AWAY': (0.3, 0.8, 1.0),
          'STANDING': (0.8, 0.8, 0.8)}


class IntentNavBridge(Node):
    def __init__(self):
        super().__init__('intent_nav_bridge')
        d = self.declare_parameter
        d('rate', 5.0)               # Hz
        d('safety_radius', 1.0)      # CPA distance (m) that counts as a conflict
        d('horizon', 2.0)            # s of future shown to the costmap
        d('step', 0.25)              # s between predicted positions
        d('human_radius', 0.25)      # m
        d('growth', 0.05)            # m of extra radius per second of prediction
        d('conflict_time', 5.0)      # only paint humans whose path meets ours within this many s
        d('grid_size', 6.0)          # m, side of the predicted-occupancy grid around the robot
        d('grid_res', 0.05)          # m/cell (same as the local costmap)
        d('max_age', 1.0)            # ignore humans not seen for this long (s)
        d('speed_floor', 0.3)        # never limit below this fraction of top speed
        d('release_rate', 0.1)       # speed limit relaxes by at most this per tick
        self.g = lambda n: self.get_parameter(n).value

        self.pose = None             # x, y, yaw, v_forward, stamp
        self.humans, self.hum_time = [], -1e9
        self.scale = 1.0

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/human_tracks', self.on_tracks, 10)
        self.pub_grid = self.create_publisher(OccupancyGrid, '/human_predicted_grid', 1)
        self.pub_limit = self.create_publisher(SpeedLimit, '/speed_limit', 5)
        self.pub_intents = self.create_publisher(String, '/human_intents', 5)
        self.pub_markers = self.create_publisher(MarkerArray, '/intent_markers', 5)
        self.create_timer(1.0 / self.g('rate'), self.step)
        self.get_logger().info('intent_nav_bridge ready (feeds Nav2, does not drive the robot)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                     msg.twist.twist.linear.x, msg.header.stamp)

    def on_tracks(self, msg):
        self.humans = json.loads(msg.data)['humans']
        self.hum_time = self.now()

    def step(self):
        if self.pose is None:
            return
        g = self.g
        x, y, yaw, vb, stamp = self.pose
        rpos = np.array([x, y])
        rvel = np.array([vb * math.cos(yaw), vb * math.sin(yaw)])
        res, n = g('grid_res'), int(round(g('grid_size') / g('grid_res')))
        origin = (math.floor((x - g('grid_size') / 2) / res) * res,
                  math.floor((y - g('grid_size') / 2) / res) * res)
        grid = np.zeros((n, n), np.int8)

        fresh = self.now() - self.hum_time < g('max_age')
        target, intents, markers = 1.0, [], MarkerArray()
        for h in (self.humans if fresh else []):
            if not h['valid'] or h['seen_age'] > g('max_age'):
                continue
            hpos = np.array([h['x'], h['y']])
            hvel = np.array([h['vx'], h['vy']])
            label, t_cpa, d_cpa = classify_intent(rpos, rvel, hpos, hvel, g('safety_radius'))
            dist = float(np.linalg.norm(hpos - rpos))
            target = min(target, speed_scale(label, t_cpa, dist, g('speed_floor')))

            # Paint the costmap only for people whose path will actually meet ours soon.
            # (Passers-by / people walking away / standing are already seen by the lidar.)
            if label in ('APPROACHING_HEAD_ON', 'CROSSING_PATH') and t_cpa <= g('conflict_time'):
                centers, radii = predicted_discs(
                    hpos, hvel, min(g('horizon'), t_cpa + 1.0), g('step'),
                    g('human_radius'), g('growth'))
                rasterize_discs(grid, origin, res, centers, radii)
            intents.append(dict(id=h['id'], label=label, speed=h['speed'],
                                heading=h['heading'], t_cpa=None if math.isinf(t_cpa) else t_cpa,
                                d_cpa=d_cpa, dist=dist))
            markers.markers += self.markers_for(h, label, hpos, hvel)
            self.get_logger().info(
                f"human #{h['id']}: {label}  speed={h['speed']:.2f} m/s  "
                f"heading={math.degrees(h['heading']):.0f} deg  dist={dist:.1f} m  "
                f"t_cpa={t_cpa:.1f}s", throttle_duration_sec=2.0)

        # slow down immediately, speed back up gradually
        self.scale = target if target < self.scale else min(target, self.scale + g('release_rate'))
        limit = SpeedLimit()
        limit.header.stamp = stamp
        limit.percentage = True
        limit.speed_limit = 100.0 * self.scale
        self.pub_limit.publish(limit)

        # Predicted occupancy. A fresh grid every cycle replaces the last one, so nothing goes stale.
        og = OccupancyGrid()
        og.header.stamp, og.header.frame_id = stamp, 'odom'
        og.info.resolution, og.info.width, og.info.height = res, n, n
        og.info.origin.position.x, og.info.origin.position.y = origin
        og.info.origin.orientation.w = 1.0
        og.data = grid.ravel().tolist()
        self.pub_grid.publish(og)

        self.pub_intents.publish(String(data=json.dumps(
            {'stamp': self.now(), 'speed_scale': self.scale, 'humans': intents})))
        self.pub_markers.publish(markers)

    def markers_for(self, h, label, hpos, hvel):
        rgb = COLORS.get(label, (0.6, 0.6, 0.6))
        text = Marker()
        text.header.frame_id = 'odom'
        text.ns, text.id, text.type = 'intent_text', int(h['id']), Marker.TEXT_VIEW_FACING
        text.pose.position.x, text.pose.position.y, text.pose.position.z = hpos[0], hpos[1], 2.0
        text.scale.z = 0.25
        text.color.r, text.color.g, text.color.b, text.color.a = (*rgb, 1.0)
        text.text = f"#{h['id']} {label} {h['speed']:.1f}m/s"
        text.lifetime.nanosec = 400_000_000

        path = Marker()
        path.header.frame_id = 'odom'
        path.ns, path.id, path.type = 'intent_path', int(h['id']), Marker.LINE_STRIP
        path.scale.x = 0.06
        path.color.r, path.color.g, path.color.b, path.color.a = (*rgb, 0.9)
        horizon = self.g('horizon') if h['speed'] > 0.2 else 0.0
        for t in (0.0, horizon):
            q = hpos + hvel * t
            path.points.append(Point(x=float(q[0]), y=float(q[1]), z=0.05))
        path.lifetime.nanosec = 400_000_000
        return [text, path]


def main():
    rclpy.init()
    node = IntentNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
