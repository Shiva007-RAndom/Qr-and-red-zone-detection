"""Red-zone avoidance for the SkyScan mission. No ROS (numpy + OpenCV only).

Pieces (used by lawnmower_mission.py, tested by test_avoidance.py):
    project_pixel   camera pixel -> world X/Y on the floor
    PoseHistory     recent drone poses, to pair a detection with where the
                    drone was when the frame was taken
    footprint       the ground a camera frame surely covers
    RedMap          the red seen so far as a grid of cells (any shape, any angle)
    relocate        move a waypoint that lies inside the red (+ margin) out of it
    Detour          go round red on a straight leg: stop before it, step
                    sideways past it, run along it, drop straight back onto the
                    leg. Only straight moves between four corners.
    Coverage        which ground the camera has actually looked at, so gaps
                    left by a detour can be flown over afterwards

World frame: X east, Y north (Gazebo). Yaw is the NED yaw (0 = north,
pi/2 = east). A box is (xmin, ymin, xmax, ymax). `room` is ((xmin, xmax),
(ymin, ymax)).
"""

import math
from collections import deque

import cv2
import numpy as np


# ---------------------------------------------------------------- projection
def project_pixel(u, v, drone, hfov, img_w, img_h):
    """Pixel (u, v) -> world (x, y) on the floor, for a camera fixed to the drone, looking down.

    drone = (x, y, alt, yaw) or (x, y, alt, yaw, roll, pitch). Image up is the drone's forward,
    image right is its right (same convention as the QR centring step). Roll and pitch (NED,
    rad) tilt the view: the ray through the pixel is rotated into the world and cut with the
    floor. Returns None if the ray does not reach the floor (it points at or above the horizon).
    """
    x, y, alt, yaw = drone[:4]
    roll, pitch = (drone[4], drone[5]) if len(drone) >= 6 else (0.0, 0.0)
    f = (img_w / 2.0) / math.tan(hfov / 2.0)                       # focal length, px
    bx, by, bz = (img_h / 2.0 - v) / f, (u - img_w / 2.0) / f, 1.0   # forward, right, down
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    y1, z1 = by * cr - bz * sr, by * sr + bz * cr                  # roll about forward
    x2, z2 = bx * cp + z1 * sp, -bx * sp + z1 * cp                 # pitch about right
    north, east, down = x2 * cy - y1 * sy, x2 * sy + y1 * cy, z2   # yaw about down
    if down < 0.2:
        return None
    k = max(alt, 0.1) / down
    return (x + east * k, y + north * k)


class PoseHistory:
    """Recent (time, x, y, alt, yaw, roll, pitch) samples; `at(t)` gives the nearest one."""

    def __init__(self, maxlen=200):
        self._h = deque(maxlen=maxlen)

    def add(self, t, x, y, alt, yaw, roll=0.0, pitch=0.0):
        self._h.append((t, x, y, alt, yaw, roll, pitch))

    def at(self, t):
        h = list(self._h)                    # a snapshot: another thread may be adding to it
        if not h:
            return None
        s = min(h, key=lambda p: abs(p[0] - t))
        return s[1:]

    def max_tilt(self, t0, t1):
        """Largest roll or pitch (rad) in the samples between t0 and t1 (0 if there are none)."""
        return max((max(abs(p[5]), abs(p[6])) for p in list(self._h) if t0 <= p[0] <= t1), default=0.0)


def nadir_pixel(roll, pitch, hfov, img_w, img_h):
    """Pixel of the ground point straight below the drone (the camera is fixed to the drone,
    so it only moves from the image centre when the drone is tilted)."""
    f = (img_w / 2.0) / math.tan(hfov / 2.0)
    return (img_w / 2.0 + f * math.tan(roll),
            img_h / 2.0 + f * math.tan(pitch) / math.cos(roll))


def footprint(drone, hfov, img_w, img_h, shrink=0.2, max_tilt=0.35):
    """World box that a frame surely covers (the largest axis-aligned box inside
    the turned image rectangle, shrunk a little), or None if the drone is tilted by more
    than max_tilt rad (the picture is then too oblique to count). drone = (x, y, alt, yaw)
    or (x, y, alt, yaw, roll, pitch); a tilt moves the box to where the image centre lands."""
    x, y, alt, yaw = drone[:4]
    if len(drone) >= 6:
        if max(abs(drone[4]), abs(drone[5])) > max_tilt:
            return None
        c = project_pixel(img_w / 2.0, img_h / 2.0, drone, hfov, img_w, img_h)
        if c is None:
            return None
        x, y = c
    a = max(alt, 0.1) * math.tan(hfov / 2.0)        # half width, along the drone's right
    b = a * img_h / img_w                           # half height, along its forward
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    bx, by = b * s + a * c, b * c + a * s           # box around the tilted rectangle
    k = min(1.0, b / (bx * s + by * c), a / (bx * c + by * s))
    hx, hy = max(bx * k - shrink, 0.0), max(by * k - shrink, 0.0)
    return (x - hx, y - hy, x + hx, y + hy)


# ----------------------------------------------------------------- geometry
def _inflate(b, m):
    return (b[0] - m, b[1] - m, b[2] + m, b[3] + m)


def _inside(p, b):
    return b[0] <= p[0] <= b[2] and b[1] <= p[1] <= b[3]


def _gap(a, b):
    dx = max(a[0] - b[2], b[0] - a[2], 0.0)
    dy = max(a[1] - b[3], b[1] - a[3], 0.0)
    return math.hypot(dx, dy)


def _union(a, b):
    return (min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3]))


def _contains(a, b, tol):
    """a contains b, within tol."""
    return (a[0] <= b[0] + tol and a[1] <= b[1] + tol
            and a[2] >= b[2] - tol and a[3] >= b[3] - tol)


def in_room(p, room):
    return room[0][0] <= p[0] <= room[0][1] and room[1][0] <= p[1] <= room[1][1]


# ------------------------------------------------------------------ red map
class RedMap:
    """The red seen so far, as a grid of `cell`-metre squares in world coordinates.

    The real shape is kept, whatever its angle: a tilted zone is not turned into a
    bigger straight box, so its empty corners stay usable (its unseen inside is filled
    with the convex hull of what was seen). Red is never forgotten.
    `version` goes up whenever the map changes. `blocked(p, margin)` is True within
    `margin` of any red; `boxes` are the bounding boxes of the separate patches.
    """

    def __init__(self, room=((0.0, 60.0), (-5.0, 45.0)), cell=0.25, pad=6.0, min_size=0.3,
                 join=1.5):
        self.cell, self.min_size, self.join = cell, min_size, join
        self.x0, self.y0 = room[0][0] - pad, room[1][0] - pad
        nx = int(math.ceil((room[0][1] - room[0][0] + 2 * pad) / cell))
        ny = int(math.ceil((room[1][1] - room[1][0] + 2 * pad) / cell))
        self.grid = np.zeros((ny, nx), np.uint8)
        self.version = 0
        self._cache, self._cache_version = {}, -1
        self._filled = None
        self.looked = None      # optional: looked(p) -> True if the camera has seen the ground at p

    # -- adding red
    def add_polygon(self, pts):
        """Mark the inside of a polygon (list of world (x, y)) as red; False if nothing new."""
        if len(pts) == 0:
            return False
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        if max(max(xs) - min(xs), max(ys) - min(ys)) < self.min_size:
            return False                                   # a speck
        poly = np.array([[(x - self.x0) / self.cell - 0.5, (y - self.y0) / self.cell - 0.5]
                         for x, y in pts])
        before = int(np.count_nonzero(self.grid))
        cv2.fillPoly(self.grid, [np.round(poly).astype(np.int32)], 1)
        if int(np.count_nonzero(self.grid)) == before:
            return False
        self.version += 1
        return True

    def add_box(self, box):
        return self.add_polygon([(box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3])])

    # -- questions
    def _cell_of(self, p):
        i = int(math.floor((p[1] - self.y0) / self.cell))
        j = int(math.floor((p[0] - self.x0) / self.cell))
        return (i, j) if 0 <= i < self.grid.shape[0] and 0 <= j < self.grid.shape[1] else None

    def _fill(self):
        """The red seen so far with the unseen inside of each patch filled in.

        Only the parts of a zone the camera has looked at are marked, so a zone seen
        from two sides has a hole in the middle. Each group of patches (within `join`
        metres of each other) is replaced by its convex hull.
        """
        r = max(int(round(self.join / self.cell)), 1)
        near = cv2.dilate(self.grid, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
        n, labels = cv2.connectedComponents(near, connectivity=8)
        out = self.grid.copy()
        for lab in range(1, n):
            ys, xs = np.nonzero((labels == lab) & (self.grid > 0))
            if len(xs) >= 3:
                cv2.fillPoly(out, [cv2.convexHull(np.column_stack([xs, ys]).astype(np.int32))], 1)
        return out

    def _mask(self, margin):
        """Boolean grid: True within `margin` of any red."""
        if self._cache_version != self.version:
            self._cache, self._cache_version = {}, self.version
            self._filled = self._fill()
        key = round(margin, 3)
        if key not in self._cache:
            r = int(round(margin / self.cell))
            if r <= 0:
                m = self._filled > 0
            else:
                k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
                m = cv2.dilate(self._filled, k) > 0
            _, labels, stats, _ = cv2.connectedComponentsWithStats(m.astype(np.uint8), connectivity=8)
            boxes = [(self.x0 + st[0] * self.cell, self.y0 + st[1] * self.cell,
                      self.x0 + (st[0] + st[2]) * self.cell, self.y0 + (st[1] + st[3]) * self.cell)
                     for st in stats]
            self._cache[key] = (m, labels, boxes)
        return self._cache[key]

    def blocked(self, p, margin):
        """True if world point p is within `margin` of red."""
        c = self._cell_of(p)
        return c is not None and bool(self._mask(margin)[0][c])

    def segment_clear(self, a, b, margin, skip=0.5):
        """True if the straight move a -> b stays more than `margin` from red.
        The first `skip` metres are not checked: they are where the drone already is."""
        n = int(math.dist(a, b) / (self.cell * 0.5))
        for k in range(1, n + 1):
            t = k / n
            if t * math.dist(a, b) < skip:
                continue
            if self.blocked((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t), margin):
                return False
        return True

    def patches(self, margin):
        """Bounding boxes (world) of the separate patches of 'red + margin'."""
        return self._mask(margin)[2][1:]

    def patches_at(self, pts, margin):
        """Bounding boxes of the patches of 'red + margin' that contain any of pts."""
        _, labels, boxes = self._mask(margin)
        hit = set()
        for p in pts:
            c = self._cell_of(p)
            if c is not None and labels[c]:
                hit.add(int(labels[c]))
        return [boxes[i] for i in sorted(hit)]

    @property
    def boxes(self):
        """Bounding boxes of the red patches themselves (for display)."""
        return self.patches(0.0)

    def nearest_clear(self, pt, margin, room, buffer=0.5):
        """Nearest spot inside the room that is at least margin + buffer from all red, or None.

        "Nearest" is by the way there, which never crosses red: it is found by spreading
        out from pt through ground that is not red itself.
        """
        m = self._mask(margin + buffer)[0]
        ys = self.y0 + (np.arange(m.shape[0]) + 0.5) * self.cell
        xs = self.x0 + (np.arange(m.shape[1]) + 0.5) * self.cell
        ok = (~m & (ys[:, None] >= room[1][0]) & (ys[:, None] <= room[1][1])
              & (xs[None, :] >= room[0][0]) & (xs[None, :] <= room[0][1]))
        c = self._cell_of(pt)
        if c is None:
            return None
        free = (self._filled == 0).astype(np.uint8)
        if self._filled[c]:                              # already inside red: any way out will do
            free[:] = 1
        front = np.zeros_like(free)
        front[c] = 1
        reached = front.copy()
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        for _ in range(max(m.shape)):
            hit = (reached > 0) & ok
            if hit.any():
                ii, jj = np.nonzero(hit)
                d2 = (xs[jj] - pt[0]) ** 2 + (ys[ii] - pt[1]) ** 2
                k = int(np.argmin(d2))
                return (float(xs[jj[k]]), float(ys[ii[k]]))
            grown = cv2.dilate(reached, kernel) & free
            if np.array_equal(grown, reached):
                break
            reached = grown
        return None


    def _distance(self):
        """Grid of the distance (m) from each cell to the nearest red."""
        self._mask(0.0)
        if self._cache.get("dist") is None:
            self._cache["dist"] = (cv2.distanceTransform((self._filled == 0).astype(np.uint8),
                                                         cv2.DIST_L2, 3) * self.cell)
        return self._cache["dist"]

    def distance_to_red(self, p):
        c = self._cell_of(p)
        return 1e9 if c is None else float(self._distance()[c])

    def escape_point(self, pt, margin, room, buffer=0.5, step=1.5):
        """A point in the room that is further from the red than pt: straight away from it.

        Looks in 24 directions and at several distances for the spot furthest from red,
        so it backs off the red instead of heading for the nearest "clear" spot, which
        may be on the other side of it. None if nothing in the room is any further away.
        """
        here = self.distance_to_red(pt)
        for only_seen in ((True, False) if self.looked else (False,)):
            best, best_key = None, None
            for k in range(24):
                ang = k * math.pi / 12.0
                for r in (0.5, 1.0, step, 2.5, 4.0, 6.0):
                    q = (pt[0] + r * math.cos(ang), pt[1] + r * math.sin(ang))
                    if not in_room(q, room) or (only_seen and not self.looked(q)):
                        continue
                    d = min(self.distance_to_red(q), margin + buffer)   # no need to go beyond the margin
                    key = (round(d, 1), -r)
                    if d > here + 0.05 and (best_key is None or key > best_key):
                        best, best_key = q, key
            if best is not None:
                return best
        return None


# --------------------------------------------------------------- waypoints
def relocate(pt, red, margin, room, buffer=0.5):
    """Return pt if it is clear of the red, else the nearest clear point in the room.

    Returns None if there is no usable spot; the caller should drop the waypoint.
    """
    if not red.blocked(pt, margin):
        return pt
    return red.nearest_clear(pt, margin, room, buffer)


def aligned_legs(a, b, tol=0.5):
    """Split a move a -> b into legs that run along X or Y only (x first)."""
    if abs(a[0] - b[0]) <= tol:
        return [(a, (a[0], b[1]))]
    if abs(a[1] - b[1]) <= tol:
        return [(a, (b[0], a[1]))]
    mid = (b[0], a[1])
    return [(a, mid), (mid, b)]


# ------------------------------------------------------------------- detour
class Detour:
    """Go round red on a straight leg a -> b, using straight moves only.

    The leg runs along X or Y. If red (plus margin) is on it, the drone goes
    through four corners in the leg's own frame (s along the leg, l sideways):
        0  (S0 - gap, 0)    stop just before the red
        1  (S0 - gap, lt)   step sideways to clear it
        2  (S1 + gap, lt)   run along it
        3  (S1 + gap, 0)    drop straight back onto the leg
    S0..S1 and lt come from the red seen so far, so the corners move as more of
    the red is seen; the drone just keeps heading for the current corner. The
    side (left or right) is chosen once, the smaller detour that stays inside
    the room, and then kept. Once past corner 3 the leg is clear again.

    target() returns (point, status):
        direct   nothing in the way: a point a little way ahead on the leg
        detour   heading for a corner
        escape   already inside the margin: the nearest clear point
        blocked  no way round inside the room: hold position
    """

    def __init__(self, margin=1.5, gap=0.8, slack=0.3, reach=0.7, carrot=2.5,
                 tolerance=0.3, hug_tolerance=0.7, min_margin=1.0, floor=0.6):
        self.margin, self.gap, self.slack = margin, gap, slack
        self.min_margin, self.floor = min(min_margin, margin), floor
        self.tolerance, self.hug_tolerance = tolerance, hug_tolerance
        self.reach, self.carrot = reach, carrot
        self.a = self.b = None
        self.reset(None, None)

    def reset(self, a, b):
        self.a, self.b = a, b
        self.side = 0           # +1 left of the leg direction, -1 right
        self.idx = None         # next corner (0..3); None while no red is in the way
        self.squeeze = False    # True: pass the red with min_margin instead of margin
        self.pinched = False

    def target(self, pos, red, room):
        """The plan from _plan, but never a straight move that passes within `floor` of red."""
        tgt, status = self._plan(pos, red, room)
        if tgt is not None and status in ("direct", "detour") \
                and not red.segment_clear(pos, tgt, self.floor):
            out = red.escape_point(pos, self.margin, room, buffer=self.slack)
            return (out, "escape") if out else (None, "blocked")
        return tgt, status

    def _plan(self, pos, red, room):
        """As described above. When the full margin leaves no good way (the drone is inside
        it, it is boxed in against the room wall, or the only way round is the far side),
        use the smaller `min_margin` instead: a closer pass is better than no pass."""
        if self.squeeze:                                 # sticky, until even min_margin fails
            res = self._target(pos, red, room, self.min_margin)
            if res[1] == "direct":
                return res
            self.squeeze, self.side, self.idx = False, 0, None
        self.pinched = False
        res = self._target(pos, red, room, self.margin)
        if ((self.pinched or res[1] in ("escape", "blocked")) and self.min_margin < self.margin
                and (self._on_leg(pos) or self.idx is not None)):
            keep = (self.side, self.idx)
            self.side, self.idx = 0, None
            alt = self._target(pos, red, room, self.min_margin)
            if alt[1] == "direct":
                self.squeeze = True
                return alt
            self.side, self.idx = keep
        return res

    def _on_leg(self, pos, tol=0.6):
        """True when pos is on (within tol of) the straight line a -> b."""
        length = math.dist(self.a, self.b)
        if length < 1e-6:
            return True
        cross = ((self.b[0] - self.a[0]) * (pos[1] - self.a[1])
                 - (self.b[1] - self.a[1]) * (pos[0] - self.a[0]))
        return abs(cross) / length <= tol

    def _target(self, pos, red, room, margin):
        a, b = self.a, self.b
        length = math.dist(a, b)
        if length < 1e-6:
            return b, "direct"
        ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
        nx, ny = -uy, ux

        def to_leg(p):
            return ((p[0] - a[0]) * ux + (p[1] - a[1]) * uy,
                    (p[0] - a[0]) * nx + (p[1] - a[1]) * ny)

        def to_world(s, l):
            return (a[0] + ux * s + nx * l, a[1] + uy * s + ny * l)

        # back out only if clearly inside the margin: tracking noise at the edge is not an
        # emergency, least of all while deliberately sidestepping along the red
        tol = self.hug_tolerance if self.idx in (1, 2) else self.tolerance
        if margin <= self.min_margin + 1e-6:             # already the closest pass allowed: no slack
            tol = 0.2
        if red.blocked(pos, margin - tol):
            out = red.escape_point(pos, margin, room, buffer=self.slack)
            return (out, "escape") if out else (None, "blocked")

        s, l = to_leg(pos)

        def leg_rect(bx):
            cs = [to_leg(c) for c in ((bx[0], bx[1]), (bx[2], bx[1]), (bx[0], bx[3]), (bx[2], bx[3]))]
            return (min(c[0] for c in cs), min(c[1] for c in cs),
                    max(c[0] for c in cs), max(c[1] for c in cs))

        rects = [leg_rect(bx) for bx in red.patches(margin)]      # every patch of red + margin
        n = int((length - max(s, 0.0)) / red.cell) + 1                    # points along the leg ahead
        line = [to_world(max(s, 0.0) + k * red.cell, 0.0) for k in range(n)] + [b]
        on_leg = [leg_rect(bx) for bx in red.patches_at(line, margin)]   # patches the leg really crosses
        cl = None
        for r in on_leg:
            cl = r if cl is None else _union(cl, r)
        if cl is None:                                   # nothing in the way
            self.idx = None
            self.side = 0
            lead = max(1.2, self.carrot - 0.5 * abs(l))  # off the leg: step back onto it
            return to_world(min(length, s + lead), 0.0), "direct"
        grew = True
        while grew:                                      # red touching the red in the way
            grew = False
            for r in rects:
                if r != cl and r[0] <= cl[2] + 0.8 and r[2] >= cl[0] - 0.8 \
                        and r[1] <= cl[3] + 0.8 and r[3] >= cl[1] - 0.8 \
                        and (r[0] < cl[0] or r[2] > cl[2] or r[1] < cl[1] or r[3] > cl[3]):
                    cl = _union(cl, r)
                    grew = True
        s0, l0, s1, l1 = cl

        def fit(sc, lt, step):
            """Slide a corner along the leg until it (and its foot on the leg) is in the room."""
            for _ in range(40):
                if in_room(to_world(sc, lt), room) and in_room(to_world(sc, 0.0), room):
                    return sc
                sc += step
            return None

        def corner_ok(sign):
            lt = l1 + self.slack if sign > 0 else l0 - self.slack
            sa, sb = fit(s0 - self.gap, lt, 0.1), fit(s1 + self.gap, lt, -0.1)
            return lt, (sa is not None and sb is not None and sa <= sb), sa, sb

        if self.side:                                    # keep the side, unless it no longer fits
            lt, ok, _, _ = corner_ok(self.side)
            if not ok and self.idx == 0:
                self.side = 0
        if not self.side:
            options = [(abs(lt), sign, lt) for sign in (1, -1)
                       for lt, ok, _, _ in [corner_ok(sign)] if ok]
            if not options:
                self.pinched = True
                return None, "blocked"
            self.side = min(options)[1]
        lt, ok, sa, sb = corner_ok(self.side)
        if not ok:
            self.pinched = True
            return None, "blocked"
        self.pinched = not corner_ok(-self.side)[1]
        s0, s1 = sa + self.gap, sb - self.gap            # (corners may have slid to stay in the room)

        if self.idx is None:                             # red just came into the picture
            beside = (self.side > 0 and l > l1 + 0.1) or (self.side < 0 and l < l0 - 0.1)
            if s >= s0 - self.gap - 0.3:
                self.idx = 2 if beside else 1
            else:
                self.idx = 0
        corners = [(s0 - self.gap, 0.0), (s0 - self.gap, lt), (s1 + self.gap, lt), (s1 + self.gap, 0.0)]
        while self.idx <= 3:                             # reached the corner we were heading for?
            i = self.idx
            if i == 0:
                done = s >= corners[0][0] - 0.3 or math.hypot(s - corners[0][0], l) < self.reach
            elif i == 1:
                done = self.side * l >= self.side * lt - 0.5
            elif i == 2:
                done = s >= corners[2][0] - 0.5
            else:
                done = abs(l) <= 0.5
            if not done:
                break
            self.idx += 1
        if self.idx > 3:                                 # round it: back on the leg
            return to_world(min(length, s + self.carrot), 0.0), "direct"
        return to_world(*corners[self.idx]), "detour"


# ----------------------------------------------------------------- coverage
class Coverage:
    """Which ground the camera has looked at, on a grid of `cell` metres."""

    def __init__(self, room, cell=0.5):
        (self.x0, x1), (self.y0, y1) = room
        self.cell = cell
        self.nx = int(math.ceil((x1 - self.x0) / cell))
        self.ny = int(math.ceil((y1 - self.y0) / cell))
        self.grid = bytearray(self.nx * self.ny)

    def _span(self, lo, hi, origin, n):
        i0 = max(int(math.floor((lo - origin) / self.cell)), 0)
        i1 = min(int(math.ceil((hi - origin) / self.cell)), n)
        return range(i0, i1)

    def mark(self, box):
        """Ground inside box (a camera footprint) has been looked at."""
        for j in self._span(box[1], box[3], self.y0, self.ny):
            row = j * self.nx
            for i in self._span(box[0], box[2], self.x0, self.nx):
                self.grid[row + i] = 1

    def looked(self, p):
        """True if the camera has looked at the ground at world point p."""
        i = int(math.floor((p[0] - self.x0) / self.cell))
        j = int(math.floor((p[1] - self.y0) / self.cell))
        return 0 <= i < self.nx and 0 <= j < self.ny and bool(self.grid[j * self.nx + i])

    def gaps(self, red, margin, tile=3.0, need=0.9):
        """Centres of tiles of the room that are less than `need` covered.
        Ground in or next to red does not count (the drone must not go there)."""
        out = []
        per = max(int(round(tile / self.cell)), 1)
        for j0 in range(0, self.ny, per):
            for i0 in range(0, self.nx, per):
                total = seen = 0
                for j in range(j0, min(j0 + per, self.ny)):
                    for i in range(i0, min(i0 + per, self.nx)):
                        c = (self.x0 + (i + 0.5) * self.cell, self.y0 + (j + 0.5) * self.cell)
                        if red is not None and red.blocked(c, margin):
                            continue
                        total += 1
                        seen += self.grid[j * self.nx + i]
                if total >= per * per // 2 and seen < need * total:
                    out.append((self.x0 + (i0 + per / 2.0) * self.cell,
                                self.y0 + (j0 + per / 2.0) * self.cell))
        return out


# -------------------------------------------------------------- setpoints
class Slew:
    """The commanded point `cur`: it moves towards the target without jerking the drone.

    It speeds up and slows down at `accel`, slows for sharp turns, stops at a target that is
    a stopping point (a detour corner or an escape point) rather than overshooting it, and never
    runs more than `max_lead` ahead of the drone. When the plan changes to avoid red it is pulled
    back to within `brake_lead` of the drone first, so the drone brakes at once instead of flying
    on towards where the old plan was heading. Works in any number of dimensions (north, east, down).
    """

    def __init__(self, speed, accel=3.0, max_lead=5.5, brake_lead=1.0):
        self.speed, self.accel, self.max_lead, self.brake_lead = speed, accel, max_lead, brake_lead
        self.cur, self.vel = None, None

    def reset(self, cur):
        self.cur, self.vel = list(cur), [0.0] * len(cur)

    def hold(self):
        self.vel = [0.0] * len(self.vel)

    def pull_back(self, drone):
        d = [c - p for c, p in zip(self.cur, drone)]
        n = math.sqrt(sum(x * x for x in d))
        if n > self.brake_lead:
            self.cur = [p + x * self.brake_lead / n for p, x in zip(drone, d)]
        self.hold()

    def step(self, tgt, drone, dt, stopping=False):
        dist = math.dist(self.cur, tgt)
        if dist > 1e-6:
            u = [(t - c) / dist for c, t in zip(self.cur, tgt)]
            v = math.sqrt(sum(x * x for x in self.vel))
            align = (sum(a * b for a, b in zip(self.vel, u)) / v) if v > 0.3 else 1.0
            want = self.speed * max(0.2, (1.0 + align) / 2.0)
            if stopping:
                want = min(want, math.sqrt(2.0 * self.accel * dist))
            dv = [want * ui - vi for ui, vi in zip(u, self.vel)]
            n = math.sqrt(sum(x * x for x in dv))
            if n > self.accel * dt:
                dv = [x * self.accel * dt / n for x in dv]
            self.vel = [vi + x for vi, x in zip(self.vel, dv)]
        else:
            self.hold()
        horizontal = math.dist(self.cur[:2], drone[:2])
        if horizontal <= self.max_lead:
            move = [x * dt for x in self.vel]
            if math.sqrt(sum(x * x for x in move)) >= dist:        # arrive exactly
                move = [t - c for c, t in zip(self.cur, tgt)]
            self.cur = [c + x for c, x in zip(self.cur, move)]
        else:
            self.vel = [x * 0.9 for x in self.vel]                   # the drone is behind: let it catch up
        return self.cur


# ---------------------------------------------------------------- navigator
class Navigator:
    """Works through the sweep waypoints, then flies over any ground left uncovered.

    update(pos, now) returns (target, status): the point to head for, or None to
    hold. status is 'direct' / 'detour' / 'escape' / 'blocked' (see Detour), or
    'done' when the sweep and the revisits are finished. The caller moves the
    drone, feeds red into `red` and camera footprints into `cover`, and stops
    when the QR matches.

    Every move is split into legs along X or Y (aligned_legs), each flown with
    Detour. A waypoint is given up when the drone has not moved 1 m in `stall_time`
    seconds (or a very generous hard cap passes); the coverage check then flies
    back over what was missed. Slow sims therefore do not cause skipped rows.
    """

    def __init__(self, route, room, red, cover, margin=1.5, speed=1.5,
                 revisit_time=150.0, max_revisits=60, min_budget=90.0, blocked_limit=10.0,
                 stall_time=40.0, carrot=2.5, log=print):
        self.stall_time = stall_time
        self.route, self.i = list(route), 0
        self.room, self.red, self.cover = room, red, cover
        red.looked = cover.looked                    # escapes prefer ground the camera has already seen
        self.margin, self.speed = margin, speed
        self.revisit_time, self.max_revisits = revisit_time, max_revisits
        self.min_budget, self.blocked_limit, self.log = min_budget, blocked_limit, log
        self.det = Detour(margin=margin, carrot=carrot)
        self.phase = "sweep"                 # sweep -> revisit -> done
        self.wp = self.goal = None
        self.legs, self.leg_i = [], 0
        self.leg_t0 = self.budget = None
        self.prog_pos, self.prog_t = (0.0, 0.0), 0.0
        self.last_goal = None
        self.blocked_t0 = None               # since when the detour has reported "blocked"
        self.visited = []                    # gap centres already flown to
        self.revisits, self.revisit_t0 = 0, None
        self.label = ""

    def _next_waypoint(self, pos, now):
        if self.phase == "sweep":
            if self.i < len(self.route):
                self.wp = self.route[self.i]
                self.label = f"waypoint {self.i + 1}/{len(self.route)}"
                self.i += 1
                return True
            self.phase = "revisit"
            self.revisit_t0 = now
        if self.phase == "revisit":
            if now - self.revisit_t0 > self.revisit_time or self.revisits >= self.max_revisits:
                self.phase = "done"
                return False
            gaps = [g for g in self.cover.gaps(self.red, self.margin)
                    if all(math.dist(g, v) > 1.5 for v in self.visited)]
            if not gaps:
                self.phase = "done"
                return False
            self.wp = min(gaps, key=lambda g: math.dist(g, pos))
            self.visited.append(self.wp)
            self.revisits += 1
            self.label = f"revisit {self.revisits}"
            self.log(f"{self.label}: ground not yet seen near world ({self.wp[0]:.1f}, {self.wp[1]:.1f}); "
                     f"{len(gaps)} gap tile(s) left")
            return True
        return False

    def _place(self, wp):
        """Where to fly instead of wp if wp is in the red (+ margin).

        Rows end at the walls, so the first choice is to slide along the row, towards
        the middle of the room, to the first clear spot: the row then still gets flown
        up to the red. Failing that, the nearest clear point.
        """
        if not self.red.blocked(wp, self.margin - 0.3):          # (a little inside the margin is fine)
            return wp
        lo, hi = self.room[0]
        sign = 1.0 if wp[0] < (lo + hi) / 2.0 else -1.0
        x = wp[0]
        while lo <= x <= hi:
            x += sign * self.red.cell
            if self.red.blocked((x, wp[1]), 0.0):                # the row runs into the red itself
                break
            if lo <= x <= hi and not self.red.blocked((x, wp[1]), self.margin + 0.3):
                return (x, wp[1])
        return relocate(wp, self.red, self.margin, self.room)

    def _start_legs(self, pos):
        start = self.last_goal if (self.last_goal and math.dist(pos, self.last_goal) < 1.5) else pos
        legs = aligned_legs(start, self.goal)
        if len(legs) == 2:                   # prefer the order whose corner is clear of red
            if self.red.blocked(legs[0][1], self.margin):
                mid = (start[0], self.goal[1])
                if not self.red.blocked(mid, self.margin):
                    legs = [(start, mid), (mid, self.goal)]
        self.legs, self.leg_i = legs, 0
        self.det.reset(*legs[0])
        length = sum(math.dist(a, b) for a, b in legs)
        self.budget = max(self.min_budget, 20.0 * length / self.speed)   # hard cap only

    def update(self, pos, now):
        for _ in range(200):                 # skip finished waypoints within one call
            if self.wp is None:
                if not self._next_waypoint(pos, now):
                    return None, "done"
                self.legs, self.goal = [], None
            goal = self._place(self.wp)
            if (goal is not None and self.goal is not None and goal != self.goal
                    and not self.red.blocked(self.goal, self.margin)):
                goal = self.goal             # keep a spot that is still clear instead of chasing the edge
            if goal is None or (self.phase == "sweep" and self.last_goal
                                and math.dist(goal, self.last_goal) < 0.5):
                self.log(f"{self.label} dropped: world ({self.wp[0]:.1f}, {self.wp[1]:.1f}) "
                         "is inside the red, or repeats the previous waypoint")
                self.wp = None
                continue
            if self.goal is None or math.dist(goal, self.goal) > 0.3:   # first time, or the red moved it
                self.goal = goal
                self._start_legs(pos)
                self.leg_t0 = self.prog_t = now
                self.prog_pos = pos
            if math.dist(pos, self.prog_pos) >= 1.0:       # moving: not stuck
                self.prog_pos, self.prog_t = pos, now
            if now - self.prog_t > self.stall_time or now - self.leg_t0 > self.budget:
                why = (f"no progress for {self.stall_time:.0f} s" if now - self.prog_t > self.stall_time
                       else f"not reached in {self.budget:.0f} s")
                self.log(f"{self.label} given up: {why} "
                         f"(drone at world ({pos[0]:.1f}, {pos[1]:.1f}))")
                self.wp = None
                continue
            a, b = self.legs[self.leg_i]
            if math.dist(pos, b) < 0.6:
                if self.leg_i + 1 < len(self.legs):
                    self.leg_i += 1
                    self.det.reset(*self.legs[self.leg_i])
                    continue
                self.last_goal = self.goal
                self.log(f"{self.label} reached: world ({self.goal[0]:.1f}, {self.goal[1]:.1f})"
                         + ("  (moved off the red)" if self.goal != self.wp else ""))
                self.wp = None
                continue
            tgt, status = self.det.target(pos, self.red, self.room)
            if status != "blocked":
                self.blocked_t0 = None
            elif self.blocked_t0 is None:
                self.blocked_t0 = now
            elif now - self.blocked_t0 > self.blocked_limit:
                self.log(f"{self.label} given up: no way round the red "
                         f"(drone at world ({pos[0]:.1f}, {pos[1]:.1f}))")
                self.wp, self.blocked_t0 = None, None
                continue
            return tgt, status
        return None, "blocked"
