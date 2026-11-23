"""
Intent maths: CPA intent classifier, predicted footprints, speed scaling, local planner.
"""
import math
import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi



# Intent prediction

def classify_intent(rpos, rvel, hpos, hvel, safety=1.0, horizon=6.0, still=0.2):
    """Classify a human's intent relative to the robot.

    Uses the closest point of approach (CPA) of the two constant-velocity
    paths. Returns (label, t_cpa, d_cpa).
    """
    rpos, rvel, hpos, hvel = (np.asarray(a, float) for a in (rpos, rvel, hpos, hvel))
    p = hpos - rpos
    v = hvel - rvel
    dist = float(np.linalg.norm(p))
    hspeed = float(np.linalg.norm(hvel))
    if hspeed < still:
        return "STANDING", float("inf"), dist
    vv = float(v @ v)
    t_cpa = -float(p @ v) / vv if vv > 1e-6 else 0.0
    if t_cpa <= 0.0:
        return "MOVING_AWAY", 0.0, dist
    t_cpa = min(t_cpa, horizon)
    d_cpa = float(np.linalg.norm(p + v * t_cpa))
    if d_cpa > safety:
        return "PASSING_BY", t_cpa, d_cpa
    rspeed = float(np.linalg.norm(rvel))
    if rspeed > 0.05:                 # head-on = walking (roughly) opposite to the robot
        head_on = float(hvel @ rvel) / (hspeed * rspeed) < -0.7
    else:                             # robot at rest: is the human walking at us?
        head_on = float(hvel @ (-p / max(dist, 1e-6))) / hspeed > 0.85
    return ("APPROACHING_HEAD_ON" if head_on else "CROSSING_PATH"), t_cpa, d_cpa



# Nav2 integration helpers: where will the human be, and how much to slow down

def footprint_points(hpos, hvel, horizon=3.0, dt=0.25, r0=0.3, growth=0.12,
                     spacing=0.08, still=0.2, v_cap=2.0):
    """Cells a human may occupy during the next `horizon` seconds.

    The human is a disc of radius r0 + growth*t swept along its constant-velocity
    path (uncertainty grows with time). Returns an (N, 2) array in the world frame.
    A person slower than `still` is treated as standing (current disc only).
    """
    hpos = np.asarray(hpos, float)
    hvel = np.asarray(hvel, float)
    sp = float(np.linalg.norm(hvel))
    if sp < still:
        hvel, steps = np.zeros(2), 1
    else:
        if sp > v_cap:
            hvel = hvel * (v_cap / sp)
        steps = int(round(horizon / dt)) + 1
    pts = []
    for k in range(steps):
        t = k * dt
        c = hpos + hvel * t
        r = r0 + growth * t
        g = np.arange(-r, r + 1e-9, spacing)
        gx, gy = np.meshgrid(g, g)
        m = gx ** 2 + gy ** 2 <= r * r
        pts.append(np.column_stack((c[0] + gx[m], c[1] + gy[m])))
    pts = np.vstack(pts)
    return np.unique(np.round(pts / spacing) * spacing, axis=0)


def predicted_discs(hpos, hvel, horizon=2.0, dt=0.25, r0=0.25, growth=0.05,
                    still=0.2, v_cap=2.0):
    """Centres (K,2) and radii (K,) of the discs a human may occupy over the next `horizon` s."""
    hpos = np.asarray(hpos, float)
    hvel = np.asarray(hvel, float)
    sp = float(np.linalg.norm(hvel))
    if sp < still:
        hvel, horizon = np.zeros(2), 0.0
    elif sp > v_cap:
        hvel = hvel * (v_cap / sp)
    t = np.arange(0.0, horizon + 1e-9, dt)
    return hpos + np.outer(t, hvel), r0 + growth * t


def rasterize_discs(grid, origin, res, centers, radii, value=100):
    """Paint discs into an occupancy grid (H, W) whose cell (0,0) corner is at `origin` (x, y)."""
    h, w = grid.shape
    for (cx, cy), r in zip(centers, radii):
        i0 = max(0, int(math.floor((cx - r - origin[0]) / res)))
        i1 = min(w, int(math.ceil((cx + r - origin[0]) / res)) + 1)
        j0 = max(0, int(math.floor((cy - r - origin[1]) / res)))
        j1 = min(h, int(math.ceil((cy + r - origin[1]) / res)) + 1)
        if i0 >= i1 or j0 >= j1:
            continue
        xs = origin[0] + (np.arange(i0, i1) + 0.5) * res
        ys = origin[1] + (np.arange(j0, j1) + 0.5) * res
        m = (xs[None, :] - cx) ** 2 + (ys[:, None] - cy) ** 2 <= r * r
        grid[j0:j1, i0:i1][m] = value
    return grid


def speed_scale(label, t_cpa, dist, floor=0.3, near=1.0):
    """Fraction (floor..1) of the robot's top speed that is safe given one human."""
    s = 1.0
    if label == 'APPROACHING_HEAD_ON':
        s = (t_cpa - 1.0) / 4.0            # full speed at t_cpa>=5 s, floor at <=1 s
    elif label == 'CROSSING_PATH':
        s = 0.5 + 0.5 * (t_cpa - 2.0) / 3.0  # half speed once they cross within 2 s
        s = max(s, 0.5)
    if dist < near:                         # anyone this close: ease off regardless
        s = min(s, 0.5 * (dist / near) + 0.2)
    return float(min(1.0, max(floor, s)))


# Intent-aware local planner 

DEFAULTS = dict(v_max=0.22, w_max=1.2, horizon=3.0, dt=0.2, robot_r=0.22,
                human_r=0.3, margin=0.1, growth=0.1, social_r=1.2,
                front_scale=0.6, w_goal=1.0, w_head=0.4, w_speed=1.0,
                w_social=1.0, w_right=0.3, n_v=6, n_w=21)


def plan(robot, goal, humans, prm=None, v_cap=None, keep_right=False):
    """Pick (v, w) by rolling out candidate arcs against PREDICTED human paths.

    robot  : (x, y, yaw)
    goal   : (gx, gy)
    humans : list of (pos[2], vel[2], moving: bool) in the world frame
    Returns ((v, w), min_clearance_m)
    """
    p = {**DEFAULTS, **(prm or {})}
    x0, y0, th0 = robot
    gx, gy = goal
    vmax = p['v_max'] if v_cap is None else min(p['v_max'], v_cap)
    dt = p['dt']
    steps = int(round(p['horizon'] / dt))

    best, best_cost = None, float('inf')
    fallback, fb_clear = (0.0, 0.0), -float('inf')

    for v in np.linspace(0.0, vmax, p['n_v']):
        for w in np.linspace(-p['w_max'], p['w_max'], p['n_w']):
            x, y, th = x0, y0, th0
            social, clear = 0.0, float('inf')
            for k in range(1, steps + 1):
                th += w * dt
                x += v * math.cos(th) * dt
                y += v * math.sin(th) * dt
                t = k * dt
                hard_r = p['robot_r'] + p['human_r'] + p['margin'] + p['growth'] * t
                for hp, hv, moving in humans:
                    rx = x - (hp[0] + hv[0] * t)
                    ry = y - (hp[1] + hv[1] * t)
                    d = math.hypot(rx, ry)
                    clear = min(clear, d - hard_r)
                    d_eff = d
                    if moving:
                        sp = math.hypot(hv[0], hv[1])
                        ux, uy = hv[0] / sp, hv[1] / sp
                        along = rx * ux + ry * uy
                        perp = -rx * uy + ry * ux
                        if along > 0:                      # in front of the walker
                            along *= p['front_scale']
                        d_eff = math.hypot(along, perp)
                    if d_eff < p['social_r']:
                        social += (p['social_r'] - d_eff) ** 2
            if clear < 0.0:
                if clear > fb_clear:
                    fb_clear, fallback = clear, (v, w)
                continue
            goal_d = math.hypot(gx - x, gy - y)
            head_err = abs(wrap(math.atan2(gy - y, gx - x) - th))
            cost = (p['w_goal'] * goal_d + p['w_head'] * head_err
                    + p['w_speed'] * (p['v_max'] - v) + p['w_social'] * social)
            if keep_right:
                cost += p['w_right'] * max(0.0, w) / p['w_max']
            if cost < best_cost:
                best_cost, best = cost, (v, w)
    if best is None:
        return fallback, fb_clear
    return best, 0.0
