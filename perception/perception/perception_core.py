"""
Perception maths: foot-point ranging, camera+lidar fusion, Kalman track.
"""
import math
import numpy as np

# Perception geometry

def foot_to_robot(u, v_foot, img_w, img_h, fx, cam_h, cam_x=0.0, cam_y=0.0,
                  max_range=8.0):
    """Project a person's foot pixel (u, v_foot) onto the ground plane.

    Assumes a level camera at height `cam_h` metres. Returns (x, y) in the
    robot base frame, or None if the point is unusable (above horizon / too far).
    """
    dy = v_foot - img_h / 2.0
    if dy < 4.0:
        return None
    z = fx * cam_h / dy                      # depth along optical axis
    if z > max_range:
        return None
    lateral = (u - img_w / 2.0) * z / fx     # +right in the image
    return z + cam_x, -lateral + cam_y       # ROS: +y is left


def robot_to_world(pose, xr, yr):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + xr * c - yr * s, y + xr * s + yr * c


def lidar_refine(ranges, a0, da, cam, theta, half_w, pr, lidar_x=-0.064):
    r = np.asarray(ranges, float)
    a = a0 + da * np.arange(r.size)
    ok = np.isfinite(r) & (r > 0.12)
    r, a = r[ok], a[ok]
    p = np.column_stack((r * np.cos(a) + lidar_x, r * np.sin(a))) - cam
    d = np.hypot(p[:, 0], p[:, 1])
    z = math.hypot(pr[0] - cam[0], pr[1] - cam[1])
    ang = (np.arctan2(p[:, 1], p[:, 0]) - theta + np.pi) % (2 * np.pi) - np.pi
    m = (np.abs(ang) <= half_w) & (np.abs(d - z) < 0.5 + 0.25 * z)
    if not m.any():
        return None
    rng = d[m].min() + 0.15                  # scan hits the near surface -> add ~body radius
    return cam[0] + rng * math.cos(theta), cam[1] + rng * math.sin(theta)


# Kalman filter (constant velocity)

class Track:
    H = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.]])

    def __init__(self, tid, xy, t):
        self.id = tid
        self.x = np.array([xy[0], xy[1], 0.0, 0.0])
        self.P = np.diag([0.25, 0.25, 1.0, 1.0])
        self.t = t
        self.last_seen = t
        self.hits = 1

    def predict(self, t, accel_sigma=0.6):
        dt = t - self.t
        if dt <= 0.0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        a, b, c = dt ** 4 / 4, dt ** 3 / 2, dt ** 2
        Q = accel_sigma ** 2 * np.array(
            [[a, 0, b, 0], [0, a, 0, b], [b, 0, c, 0], [0, b, 0, c]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def update(self, xy, sigma, t, accel_sigma=0.6):
        self.predict(t, accel_sigma)
        R = np.eye(2) * sigma ** 2
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(xy, float) - self.H @ self.x)
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.last_seen = t
        self.hits += 1

    def extrapolate(self, t):
        dt = max(0.0, t - self.t)
        return (self.x[0] + self.x[2] * dt, self.x[1] + self.x[3] * dt,
                self.x[2], self.x[3])

    @property
    def speed(self):
        return float(math.hypot(self.x[2], self.x[3]))

    @property
    def heading(self):
        return float(math.atan2(self.x[3], self.x[2]))
