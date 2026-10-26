#!/usr/bin/env python3
"""Offline tests for avoidance.py (no ROS, no Gazebo):  python3 launch/test_avoidance.py"""

import math
from collections import deque

from avoidance import (Coverage, Slew, nadir_pixel, Detour, Navigator, PoseHistory, RedMap, aligned_legs,
                       footprint, in_room, project_pixel, relocate)

ROOM = ((14.0, 52.0), (2.5, 37.5))
HFOV, W, H = 1.74, 1280, 960
MARGIN = 1.5                  # used by the unit tests
SWEEP_MARGIN = 2.0            # the mission's default, used by the flight simulations
ZONE_SIZE = 8.0
QRS = {"qr_a": (20.0, 32.0), "qr_b": (18.0, 8.0), "qr_c": (24.0, 4.0), "qr_d": (46.0, 7.5)}


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


def R(*boxes):
    """A red map holding these (xmin, ymin, xmax, ymax) boxes."""
    m = RedMap(ROOM)
    for b in boxes:
        assert m.add_box(b)
    return m


# ------------------------------------------------------------ unit checks
def test_projection():
    for yaw in (0.0, 1.0, math.pi / 2):                       # centre pixel is straight below
        x, y = project_pixel(W / 2, H / 2, (10.0, 20.0, 3.0, yaw), HFOV, W, H)
        assert approx(x, 10.0) and approx(y, 20.0)
    mpp = 2 * 3.0 * math.tan(HFOV / 2) / W
    x, y = project_pixel(W / 2, H / 2 - 100, (0.0, 0.0, 3.0, math.pi / 2), HFOV, W, H)
    assert approx(x, 100 * mpp) and approx(y, 0.0, 1e-9)      # facing east: up = east
    x, y = project_pixel(W / 2 + 100, H / 2, (0.0, 0.0, 3.0, math.pi / 2), HFOV, W, H)
    assert approx(x, 0.0, 1e-9) and approx(y, -100 * mpp)     # right = south
    x, y = project_pixel(W / 2 + 100, H / 2 - 100, (0.0, 0.0, 3.0, 0.0), HFOV, W, H)
    assert approx(x, 100 * mpp) and approx(y, 100 * mpp)      # facing north: up = north


def test_projection_with_tilt():
    d = (10.0, 20.0, 10.0, math.pi / 2)                     # level, facing east, 10 m up
    assert all(approx(a, b, 1e-6) for a, b in zip(project_pixel(640, 480, d + (0.0, 0.0), HFOV, W, H),
                                                  project_pixel(640, 480, d, HFOV, W, H)))
    nose_up = project_pixel(640, 480, d + (0.0, math.radians(10)), HFOV, W, H)
    assert nose_up[0] > 10.0 + 1.5 and approx(nose_up[1], 20.0, 1e-6)     # nose up: belly camera looks ahead
    nose_dn = project_pixel(640, 480, d + (0.0, -math.radians(10)), HFOV, W, H)
    assert nose_dn[0] < 10.0 - 1.5
    right_dn = project_pixel(640, 480, d + (math.radians(10), 0.0), HFOV, W, H)
    assert right_dn[1] > 20.0 + 1.5 and approx(right_dn[0], 10.0, 1e-6)  # right wing down: looks left (north)
    assert project_pixel(640, 0, d + (0.0, math.radians(60)), HFOV, W, H) is None    # sees the sky
    assert footprint(d + (0.0, math.radians(40)), HFOV, W, H) is None                # too oblique
    fp = footprint(d + (0.0, math.radians(10)), HFOV, W, H)
    assert fp is not None and (fp[0] + fp[2]) / 2 > 11.0                              # moved ahead


def test_slew():
    s = Slew(speed=5.0, accel=3.0, max_lead=5.5)
    s.reset([0.0, 0.0, -10.0])
    drone = [0.0, 0.0, -10.0]
    for _ in range(40):                                   # 2 s at 0.05 s: accelerates, never above accel
        s.step([100.0, 0.0, -10.0], list(s.cur), 0.05)
    assert 4.0 < s.cur[0] < 7.5 and approx(math.hypot(*s.vel), 5.0, 0.05)   # capped at the speed
    far = [s.cur[0] - 10.0, 0.0, -10.0]                   # the drone is 10 m behind: setpoint waits
    x0 = s.cur[0]
    s.step([100.0, 0.0, -10.0], far, 0.05)
    assert approx(s.cur[0], x0)
    s.pull_back(drone)
    assert math.dist(s.cur[:2], drone[:2]) <= 1.0 + 1e-9 and s.vel == [0.0, 0.0, 0.0]
    t = Slew(speed=5.0, accel=3.0, max_lead=50.0)         # stops at a stopping target
    t.reset([0.0, 0.0])
    for _ in range(2000):
        t.step([20.0, 0.0], [t.cur[0], 0.0], 0.05, stopping=True)
    assert approx(t.cur[0], 20.0, 0.05) and math.hypot(*t.vel) < 0.5


def test_nadir_pixel():
    assert nadir_pixel(0.0, 0.0, HFOV, W, H) == (W / 2.0, H / 2.0)
    u, v = nadir_pixel(0.0, math.radians(10), HFOV, W, H)          # nose up: the ground below is behind
    assert approx(u, W / 2.0) and v > H / 2.0 + 50
    d = (10.0, 20.0, 10.0, math.pi / 2, 0.0, math.radians(10))
    q = project_pixel(*nadir_pixel(0.0, math.radians(10), HFOV, W, H), d, HFOV, W, H)
    assert approx(q[0], 10.0, 1e-6) and approx(q[1], 20.0, 1e-6)   # that pixel really is straight below
    d = (10.0, 20.0, 10.0, 0.3, math.radians(8), math.radians(-5))
    q = project_pixel(*nadir_pixel(math.radians(8), math.radians(-5), HFOV, W, H), d, HFOV, W, H)
    assert approx(q[0], 10.0, 1e-6) and approx(q[1], 20.0, 1e-6)


def test_pose_history():
    h = PoseHistory()
    assert h.at(0) is None
    h.add(1.0, 1, 1, 3, 0)
    h.add(2.0, 2, 2, 3, 0)
    assert h.at(1.9)[0] == 2 and h.at(1.1)[0] == 1


def test_footprint():
    fp = footprint((10.0, 20.0, 3.0, math.pi / 2), HFOV, W, H)      # facing east
    assert approx(fp[0], 10 - 2.47, 0.02) and approx(fp[2], 10 + 2.47, 0.02)
    assert approx(fp[1], 20 - 3.36, 0.02) and approx(fp[3], 20 + 3.36, 0.02)


def test_redmap():
    m = RedMap(ROOM)
    assert not m.add_polygon([(0, 0), (0.1, 0.1)])            # speck
    assert m.add_polygon([(30, 10), (34, 10), (34, 14), (30, 14)])
    v = m.version
    assert not m.add_polygon([(30.5, 10.5), (33, 13), (33, 10.5)])    # already red
    assert m.version == v
    assert m.add_polygon([(33, 13), (37, 13), (37, 18), (33, 18)])   # overlaps: grows
    assert len(m.boxes) == 1
    assert m.add_polygon([(20, 30), (24, 30), (24, 33), (20, 33)])   # far away: new patch
    assert len(m.boxes) == 2
    assert m.blocked((32, 12), 0.0) and not m.blocked((26, 12), 0.0)
    assert m.blocked((28.5, 12), 2.0) and not m.blocked((27.5, 12), 2.0)       # 2 m margin


def test_tilted_zone_keeps_its_shape():
    """A diamond is not turned into its bounding box: the empty corners stay usable."""
    m = RedMap(ROOM)
    c, h = (32.0, 20.0), 5.0
    assert m.add_polygon([(c[0], c[1] - h), (c[0] + h, c[1]), (c[0], c[1] + h), (c[0] - h, c[1])])
    assert m.blocked(c, 0.0)
    assert not m.blocked((c[0] + 4.0, c[1] + 4.0), 0.0)                 # corner of its bounding box
    assert not m.blocked((c[0] + 4.0, c[1] + 4.0), 1.5)                 # still clear with a margin
    assert m.blocked((c[0] + 4.0, c[1] + 4.0), 3.0)
    assert len(m.boxes) == 1 and approx(m.boxes[0][2] - m.boxes[0][0], 2 * h, 0.6)


def test_unseen_inside_is_filled():
    """A zone seen only along two edges is treated as solid."""
    m = RedMap(ROOM)
    m.add_box((34.0, 4.5, 36.0, 12.5))                     # its east edge
    m.add_box((28.0, 11.0, 36.0, 12.5))                    # its north edge
    assert m.blocked((31.0, 8.0), 0.0)                     # the unseen middle
    assert not m.blocked((24.0, 8.0), 1.0)
    far = RedMap(ROOM)
    far.add_box((20.0, 10.0, 22.0, 12.0))
    far.add_box((40.0, 10.0, 42.0, 12.0))                  # separate zones stay separate
    assert not far.blocked((31.0, 11.0), 0.0)


def test_relocate():
    red = R((29.0, 10.0, 37.0, 18.0))
    assert relocate((20.0, 14.0), red, MARGIN, ROOM) == (20.0, 14.0)    # clear
    p = relocate((33.0, 14.0), red, MARGIN, ROOM)                       # in the middle
    assert p is not None and not red.blocked(p, MARGIN) and in_room(p, ROOM)
    p = relocate((28.0, 14.0), red, MARGIN, ROOM)                       # in the margin
    assert p is not None and p[0] < 27.6 and abs(p[1] - 14.0) < 0.6 and in_room(p, ROOM)
    wall = R((14.0, 10.0, 22.0, 18.0))                                  # against the west wall
    p = relocate((15.0, 14.0), wall, MARGIN, ROOM)
    assert p is not None and in_room(p, ROOM) and not wall.blocked(p, MARGIN)
    assert relocate((30.0, 20.0), R((14.0, 2.5, 52.0, 37.5)), MARGIN, ROOM) is None


def test_aligned_legs():
    assert aligned_legs((0, 0), (0, 5)) == [((0, 0), (0, 5))]
    assert aligned_legs((0, 0), (5, 0.2)) == [((0, 0), (5, 0))]
    assert aligned_legs((0, 0), (5, 5)) == [((0, 0), (5, 0)), ((5, 0), (5, 5))]


def test_detour_corners():
    zone_box = R((28.0, 4.5, 36.0, 12.5))        # inflated: x 26.5..37.5, y 3.0..14.0
    d = Detour(margin=MARGIN)
    d.reset((52.0, 7.5), (14.0, 7.5))           # westbound along y = 7.5
    t, s = d.target((50.0, 7.5), zone_box, ROOM)
    assert s == "detour" and approx(t[0], 37.5 + 0.8, 0.4) and approx(t[1], 7.5, 0.4)   # stop before it
    side = d.side
    t, s = d.target((38.3, 7.5), zone_box, ROOM)                                     # at the corner
    assert s == "detour" and approx(t[0], 38.3, 0.4) and abs(t[1] - 7.5) > 3.0      # step sideways
    y_side = t[1]
    t, s = d.target((38.3, y_side), zone_box, ROOM)                                  # sideways done
    assert s == "detour" and approx(t[1], y_side, 0.4) and approx(t[0], 25.7, 0.4)  # run along
    t, s = d.target((27.0, y_side), zone_box, ROOM)                                  # still beside it
    assert s == "detour" and approx(t[0], 25.7, 0.4) and approx(t[1], y_side, 0.4)
    t, s = d.target((26.0, y_side), zone_box, ROOM)                                  # past it: straight back
    assert s == "direct" and approx(t[1], 7.5, 0.4) and t[0] < 26.0
    t, s = d.target((25.0, 7.5 + (y_side - 7.5) * 0.1), zone_box, ROOM)              # nearly on the row
    assert s == "direct" and approx(t[1], 7.5, 0.4) and t[0] < 25.0
    assert side != 0 and d.side == 0                                                 # side is chosen per zone
    d.reset((14.0, 20.0), (52.0, 20.0))                                              # nothing in the way
    t, s = d.target((30.0, 20.0), zone_box, ROOM)
    assert s == "direct" and approx(t[1], 20.0) and t[0] > 30.0
    t, s = d.target((27.5, 20.0), R((28.0, 15.0, 36.0, 23.0)), ROOM)                  # well inside the margin
    assert s == "escape" and t[0] < 26.5
    d.reset((14.0, 20.0), (52.0, 20.0))                                              # no way round
    t, s = d.target((20.0, 20.0), R((25.0, 0.0, 30.0, 40.0)), ROOM)
    assert t is None and s == "blocked"


def test_detour_stays_in_the_room():
    d = Detour(margin=MARGIN)
    d.reset((52.0, 36.0), (14.0, 36.0))                     # a row near the north limit
    t, s = d.target((45.0, 36.0), R((28.0, 33.0, 36.0, 41.0)), ROOM)
    assert s == "detour" and d.side != 0 and in_room(t, ROOM)


def test_coverage():
    c = Coverage(ROOM)
    assert len(c.gaps(None, MARGIN)) > 100                    # nothing seen yet
    c.mark((14.0, 2.5, 52.0, 37.5))
    assert c.gaps(None, MARGIN) == []
    c = Coverage(ROOM)
    c.mark((14.0, 2.5, 52.0, 20.0))                         # only the south half
    gaps = c.gaps(None, MARGIN)
    assert gaps and all(g[1] >= 18.9 for g in gaps)
    c = Coverage(ROOM)                                      # ground in or next to red is not a gap
    c.mark((14.0, 2.5, 26.5, 37.5))                         # everything except the red + margin
    c.mark((37.5, 2.5, 52.0, 37.5))
    c.mark((26.5, 14.0, 37.5, 37.5))
    c.mark((26.5, 2.5, 37.5, 3.0))
    assert c.gaps(R((28.0, 4.5, 36.0, 12.5)), MARGIN) == []
    assert c.gaps(None, MARGIN) != []                         # without the red it would be a gap


# ------------------------------------------------- whole-sweep simulation
def serpentine(row_spacing=5.0):
    """The mission route with no hard-coded zone."""
    route = [(2.0, 21.75), (13.0, 21.75), (ROOM[0][0], 21.75)]
    ys, y = [], ROOM[1][0]
    while y <= ROOM[1][1] + 1e-6:
        ys.append(y)
        y += row_spacing
    route.append((ROOM[0][0], ys[0]))
    east = True
    for y in ys:
        xs, xe = ROOM[0] if east else ROOM[0][::-1]
        route += [(xs, y), (xe, y)]
        east = not east
    return route


def zone(cx, cy, size=ZONE_SIZE, deg=0.0):
    """A square red zone as a polygon, turned by `deg` degrees."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    h = size / 2
    return [(cx + x * c - y * s, cy + x * s + y * c) for x, y in ((-h, -h), (h, -h), (h, h), (-h, h))]


def bbox(poly):
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    return (min(xs), min(ys), max(xs), max(ys))


def clip_to_box(poly, box):
    """Part of a convex polygon inside an axis-aligned box (Sutherland-Hodgman)."""
    def cut(pts, inside, cross):
        out = []
        for i, q in enumerate(pts):
            pr = pts[i - 1]
            if inside(q):
                if not inside(pr):
                    out.append(cross(pr, q))
                out.append(q)
            elif inside(pr):
                out.append(cross(pr, q))
        return out
    x0, y0, x1, y1 = box
    def at_x(x):
        return lambda a, b: (x, a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]))
    def at_y(y):
        return lambda a, b: (a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]), y)
    for inside, cross in ((lambda q: q[0] >= x0, at_x(x0)), (lambda q: q[0] <= x1, at_x(x1)),
                          (lambda q: q[1] >= y0, at_y(y0)), (lambda q: q[1] <= y1, at_y(y1))):
        if not poly:
            break
        poly = cut(poly, inside, cross)
    return poly


def dist_to_zone(pt, poly):
    """Distance from a point to a convex polygon (0 inside)."""
    n, inside, best = len(poly), True, 1e9
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ex, ey = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((pt[0] - a[0]) * ex + (pt[1] - a[1]) * ey) / (ex * ex + ey * ey)))
        best = min(best, math.hypot(pt[0] - a[0] - t * ex, pt[1] - a[1] - t * ey))
        if ex * (pt[1] - a[1]) - ey * (pt[0] - a[0]) < 0:
            inside = False
    return 0.0 if inside else best


def fly(zones, speed=1.5, dt=0.05, alt=3.0, lag=0.9, stop_on=None, drone_speed=None, qrs=None,
        map_err=(0.0, 0.0), noise=0.0, rng=None, max_steps=400000, dynamic=False,
        accel=3.0, drone_accel=5.0, margin=SWEEP_MARGIN):
    """Fly the whole mission against true zones, seeing only what the camera shows.

    The camera sees the part of each zone inside its footprint, and the
    information is `lag` seconds old by the time it reaches the planner, as in
    the real system. With dynamic=True the drone follows the commanded point the way PX4 does
    (a P controller on position error with an acceleration limit, so it lags the setpoint), and the
    setpoint comes from the same Slew class the mission uses. Returns closest approach, QRs that were in view, how far
    the drone strayed from the row after passing the first zone, and the log.
    """
    yaw = math.pi / 2                                  # facing east
    half_r = alt * math.tan(HFOV / 2)                  # half width of the image on the ground
    half_f = half_r * H / W
    log = []
    red, cover = RedMap(ROOM), Coverage(ROOM)
    nav = Navigator(serpentine(), ROOM, red, cover, margin=margin, speed=speed, revisit_time=300.0,
                    carrot=max(2.5, 1.1 * speed + 1.0) if dynamic else 2.5, log=log.append)
    pos, t = (2.0, 20.0), 0.0
    hist = deque([pos] * max(int(lag / dt), 1), maxlen=max(int(lag / dt), 1))
    worst, seen, off_row, steps = 1e9, set(), 0.0, 0
    slew, prev_status, vel = None, "direct", [0.0, 0.0]
    row0_y, vmax = 0.0, drone_speed or speed
    while True:
        p = hist[0]                                     # where the frame being processed was taken
        for z in zones:
            vis = clip_to_box(z, (p[0] - half_f, p[1] - half_r, p[0] + half_f, p[1] + half_r))
            if len(vis) >= 3:
                red.add_polygon([(q[0] + map_err[0], q[1] + map_err[1]) for q in vis])
        fp = footprint((p[0], p[1], alt, yaw), HFOV, W, H, shrink=0.4)
        cover.mark(fp)
        for name, q in (qrs or QRS).items():
            if fp[0] <= q[0] <= fp[2] and fp[1] <= q[1] <= fp[3]:
                seen.add(name)
        if stop_on and stop_on in seen:
            break
        seen_pos = pos if not noise else (pos[0] + rng.gauss(0, noise), pos[1] + rng.gauss(0, noise))
        tgt, status = nav.update(seen_pos, t)
        if status == "done":
            break
        if dynamic:
            if slew is None:
                slew = Slew(speed, accel, max(1.0, 1.1 * speed))
                slew.reset(pos)
            if tgt is None:
                slew.pull_back(pos)
            else:
                if status != prev_status and status in ("detour", "escape"):
                    slew.pull_back(pos)
                elif status == "escape":
                    slew.pull_back(pos)
                slew.step(tgt, pos, dt, stopping=status in ("detour", "escape"))
            prev_status = status
            want = [min(0.95 * (c - p), 12.0) if c >= p else max(0.95 * (c - p), -12.0)
                    for c, p in zip(slew.cur, pos)]
            dv = [w - v for w, v in zip(want, vel)]
            n = math.hypot(*dv)
            if n > drone_accel * dt:
                dv = [q * drone_accel * dt / n for q in dv]
            vel = [v + q for v, q in zip(vel, dv)]
            pos = (pos[0] + vel[0] * dt, pos[1] + vel[1] * dt)
        elif tgt is not None:
            d = math.dist(pos, tgt)
            st = min(vmax * dt, d)
            if d > 1e-9:
                pos = (pos[0] + (tgt[0] - pos[0]) * st / d, pos[1] + (tgt[1] - pos[1]) * st / d)
        hist.append(pos)
        if nav.label == "waypoint 6/20":
            row0_y = max(row0_y, pos[1])
        t += dt
        steps += 1
        assert steps < max_steps, "mission did not finish"
        for z in zones:
            worst = min(worst, dist_to_zone(pos, z))
        # on the row-7.5 leg, once clear of the first zone: how far from the row?
        if zones and nav.label == "waypoint 8/20" and nav.wp == (14.0, 7.5):
            z = bbox(zones[0])
            if z[1] - SWEEP_MARGIN <= 7.5 <= z[3] + SWEEP_MARGIN and pos[0] < z[0] - SWEEP_MARGIN - 3.0:
                off_row = max(off_row, abs(pos[1] - 7.5))
    return dict(closest=worst, seen=seen, off_row=off_row, time=t, log=log, row0_y=row0_y)


def test_sweeps():
    cases = {
        "world zone (32,8.5)": ([zone(32, 8.5)], True),
        "previous position (33,10)": ([zone(33, 10)], True),
        "old position (33,14)": ([zone(33, 14)], True),
        "zone moved (25,25)": ([zone(25, 25)], True),
        "zone far east (44,30)": ([zone(44, 30)], True),
        "zone on a row end (49,20)": ([zone(49, 20)], True),
        "two zones": ([zone(26, 12, 5), zone(44, 28, 6)], False),
        "no zone": ([], True),
    }
    for name, (zones, all_qrs) in cases.items():
        r = fly(zones)
        missing = sorted(set(QRS) - r["seen"])
        print(f"  {name:28s} closest {r['closest']:5.2f} m  {r['time']:5.0f} s  "
              f"QRs not seen: {missing or 'none'}  off the row after the zone: {r['off_row']:.1f} m")
        assert r["closest"] > 1.0, f"{name}: came within {r['closest']:.2f} m of the real zone"
        if all_qrs:
            assert not missing, f"{name}: never saw {missing}"


def test_correct_qr_is_seen_with_the_world_zone():
    """The failure from the real run: the drone left row 7.5 and passed qr_b 4 m away."""
    r = fly([zone(32, 8.5)], stop_on="qr_b")
    assert "qr_b" in r["seen"]
    assert r["off_row"] < 1.0, f"{r['off_row']:.1f} m off the row after the zone"
    assert not any("given up" in m for m in r["log"]), r["log"]


def test_row_beside_the_zone_is_squeezed_past():
    """The world zone is 2 m from row 2.5: pass it closely, do not go round the north side."""
    r = fly([zone(32, 8.5)])
    assert r["row0_y"] < 8.0, f"row 2.5 swung out to y={r['row0_y']:.1f}"
    assert r["closest"] > 1.0


def test_slow_drone_does_not_skip_rows():
    """The real run: the drone moved ~0.3 m/s, legs timed out and row 7.5 was skipped."""
    r = fly([zone(32, 8.5)], drone_speed=0.3, stop_on="qr_b")
    assert "qr_b" in r["seen"], r["log"]
    assert not any("given up" in m for m in r["log"]), r["log"]


def corner_qrs(poly, far=2.6, near=1.6):
    """QR tiles in the empty corners of a tilted zone's bounding box: one clear of the
    keep-out (far) and one inside the margin (near). Chosen from the box corners."""
    x0, y0, x1, y1 = bbox(poly)
    out = {}
    for cx, cy, name in ((x0, y0, "sw"), (x1, y0, "se"), (x0, y1, "nw"), (x1, y1, "ne")):
        sx, sy = (1 if cx == x0 else -1), (1 if cy == y0 else -1)
        for k in range(0, 80):                           # walk from the corner towards the centre
            q = (cx + sx * k * 0.1, cy + sy * k * 0.1)
            d = dist_to_zone(q, poly)
            if d <= far and name + "_far" not in out and d >= far - 0.2:
                out[name + "_far"] = q
            if d <= near and name + "_near" not in out and d >= near - 0.2:
                out[name + "_near"] = q
    return out


def test_tilted_zones():
    """Zones at awkward angles, QR tiles in the empty corners of their bounding boxes."""
    for deg in (20, 35, 45, 60):
        poly = zone(32, 9.5, deg=deg)
        tiles = corner_qrs(poly)
        assert tiles, "no test tiles found"
        qrs = {**QRS, **tiles}
        r = fly([poly], qrs=qrs)
        missing = sorted(set(qrs) - r["seen"])
        print(f"  tilted {deg:2d} deg: closest {r['closest']:5.2f} m  tiles {len(tiles)}  "
              f"not seen: {missing or 'none'}")
        assert r["closest"] > 0.9, f"{deg} deg: came within {r['closest']:.2f} m"
        assert not missing, f"{deg} deg: never saw {missing}"


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    for n, f in tests:
        print(n)
        f()
    print(f"{len(tests)} tests passed")
