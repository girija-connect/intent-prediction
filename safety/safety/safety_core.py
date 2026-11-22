import math
import numpy as np

# Safety zones (yellow = slow down, red = stop)

def zone_state(pts, direction, red_r, yellow_r, self_r=0.0, half_angle=math.pi,
               min_points=1, prev='clear', hyst=0.0):
    """Which zone are the lidar points (N,2 in base_link) in?

    direction : heading (rad) the robot moves toward, or None to look all around.
    Returns (state, nearest) with state in 'clear' | 'yellow' | 'red'.
    """
    pts = np.asarray(pts, float).reshape(-1, 2)
    d = np.hypot(pts[:, 0], pts[:, 1])
    keep = d > self_r
    if direction is not None:
        ang = np.arctan2(pts[:, 1], pts[:, 0]) - direction
        ang = (ang + np.pi) % (2 * np.pi) - np.pi
        keep &= np.abs(ang) <= half_angle
    d = d[keep]
    nearest = float(d.min()) if d.size else float('inf')
    red_t = red_r + (hyst if prev == 'red' else 0.0)
    yel_t = yellow_r + (hyst if prev in ('yellow', 'red') else 0.0)
    if int(np.sum(d < red_t)) >= min_points:
        return 'red', nearest
    if int(np.sum(d < yel_t)) >= min_points:
        return 'yellow', nearest
    return 'clear', nearest
