#!/usr/bin/env python3
"""
Detect walking humans in the camera image and estimate their world-frame position, speed and heading.

Pipeline: YOLOv8 + ByteTrack (IDs) -> foot pixel -> ground-plane position -> odom frame -> per-human Kalman filter -> speed / heading.
"""
import json
import math

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .perception_core import Track, foot_to_robot, lidar_refine, robot_to_world


class HumanTracker(Node):
    def __init__(self):
        super().__init__('human_tracker')
        d = self.declare_parameter
        d('image_topic', '/camera/image_raw')
        d('odom_topic', '/odom')
        d('model', 'yolov8n.pt')
        d('conf', 0.4)
        d('hfov', 1.085595)       # horizontal FOV (rad)
        d('cam_height', 0.220)    # camera height above ground (m)
        d('cam_x', -0.064)        # camera offset from base_link (m, forward)
        d('cam_y', 0.0)           # camera offset from base_link (m, left)
        d('max_range', 8.0)
        d('accel_sigma', 0.6)     # how erratically people may change velocity
        d('track_ttl', 2.0)       # keep predicting an unseen human this long (s)
        d('min_hits', 6)          # detections before speed/heading is trusted
        d('publish_debug', True)
        self.g = lambda n: self.get_parameter(n).value

        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise SystemExit('Install ultralytics:  pip install ultralytics') from e
        self.model = YOLO(self.g('model'))
        self.bridge = CvBridge()
        self.tracks = {}
        self.pose = self.scan = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, self.g('image_topic'), self.on_image, qos)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos)
        self.create_subscription(Odometry, self.g('odom_topic'), self.on_odom, 10)
        self.pub_tracks = self.create_publisher(String, '/human_tracks', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/human_markers', 10)
        self.pub_debug = self.create_publisher(Image, '/human_tracker/debug_image', 1)
        self.create_timer(0.1, self.publish)
        self.get_logger().info('human_tracker ready')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_image(self, msg):
        if self.pose is None:
            return
        g = self.g
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]
        fx = (w / 2.0) / math.tan(g('hfov') / 2.0)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self.pose

        res = self.model.track(frame, persist=True, classes=[0], conf=g('conf'),
                               tracker='bytetrack.yaml', verbose=False)[0]
        if res.boxes.id is not None:
            ids = res.boxes.id.int().cpu().tolist()
            boxes = res.boxes.xyxy.cpu().numpy()
            for tid, (x1, y1, x2, y2) in zip(ids, boxes):
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                if y2 > h - 3:          # feet cut off -> ground-plane depth invalid
                    continue
                pr = foot_to_robot((x1 + x2) / 2.0, y2, w, h, fx, g('cam_height'),
                                   g('cam_x'), g('cam_y'), g('max_range'))
                if pr is None:
                    continue
                sigma = max(0.10, 0.06 * pr[0])      # depth error grows with range
                s = self.scan                        # lidar replaces the foot-point range
                hit = s and lidar_refine(s.ranges, s.angle_min, s.angle_increment,
                                         (g('cam_x'), g('cam_y')),
                                         -math.atan(((x1 + x2) / 2 - w / 2) / fx),
                                         math.atan((x2 - x1) / 4 / fx), pr)
                if hit:
                    pr, sigma = hit, 0.05
                wx, wy = robot_to_world(pose, *pr)
                tr = self.tracks.get(tid)
                if tr is None:
                    self.tracks[tid] = Track(tid, (wx, wy), t)
                else:
                    tr.update((wx, wy), sigma, t, g('accel_sigma'))
                    label = f'#{tid}{"L" if hit else ""} {tr.speed:.2f}m/s {math.degrees(tr.heading):.0f}deg'
                    cv2.putText(frame, label, (int(x1), max(15, int(y1) - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        if g('publish_debug'):
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8', header=msg.header))

    def publish(self):
        now = self.now()
        g = self.g
        humans, markers = [], MarkerArray()
        for tid, tr in list(self.tracks.items()):
            if now - tr.last_seen > g('track_ttl'):
                del self.tracks[tid]
                continue
            x, y, vx, vy = tr.extrapolate(now)
            valid = tr.hits >= g('min_hits')
            humans.append(dict(id=int(tid), x=float(x), y=float(y), vx=float(vx),
                               vy=float(vy), speed=tr.speed, heading=tr.heading,
                               valid=bool(valid), seen_age=float(now - tr.last_seen)))
            if valid:
                m = Marker()
                m.header.frame_id = 'odom'
                m.ns, m.id, m.type = 'human_vel', int(tid), Marker.ARROW
                m.scale.x, m.scale.y, m.scale.z = 0.06, 0.12, 0.12
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 1.0
                m.points = [Point(x=float(x), y=float(y), z=0.1),
                            Point(x=float(x + vx), y=float(y + vy), z=0.1)]
                m.lifetime.nanosec = 300_000_000
                markers.markers.append(m)
        self.pub_tracks.publish(String(data=json.dumps({'stamp': now, 'humans': humans})))
        self.pub_markers.publish(markers)


def main():
    rclpy.init()
    node = HumanTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()