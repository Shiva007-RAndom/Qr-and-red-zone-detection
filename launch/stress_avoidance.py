#!/usr/bin/env python3
"""Randomised stress test for the red-zone avoidance (no ROS, no Gazebo).

    python3 launch/stress_avoidance.py [N scenarios] [first seed]

Each scenario draws 1-3 axis-aligned red zones (some touching the walls), four QR tiles
outside them, a camera delay, a drone speed, a systematic map error and position noise,
then flies the whole mission with test_avoidance.fly. A scenario FAILS if the drone
enters red, never finishes, or misses a tile that is at least 2.5 m clear of every zone.
"""

import math
import os
import multiprocessing as mp
import random
import sys
import time

import test_avoidance as T

ROOM = T.ROOM


def rect_dist(p, z):
    return math.hypot(max(z[0] - p[0], p[0] - z[2], 0.0), max(z[1] - p[1], p[1] - z[3], 0.0))


def scenario(seed):
    r = random.Random(seed)
    zones = []
    for _ in range(r.choice([1, 1, 2, 2, 3])):
        for _try in range(50):
            w, h = r.uniform(3, 12), r.uniform(3, 12)
            x0 = r.uniform(ROOM[0][0] - 1, ROOM[0][1] - w + 1)       # may touch the west/east wall
            y0 = r.uniform(ROOM[1][0] - 2, ROOM[1][1] - h + 2)
            z = (x0, y0, x0 + w, y0 + h)
            if z[0] < 20.0 and z[3] > 14.0 and z[1] < 26.0:      # would wall off the corridor mouth
                continue
            if all(max(z[0] - q[2], q[0] - z[2], z[1] - q[3], q[1] - z[3]) > 3.0 for q in zones):
                zones.append(z)                                        # zones at least 3 m apart
                break
    qrs = {}
    while len(qrs) < 4:
        q = (r.uniform(ROOM[0][0] + 1, ROOM[0][1] - 1), r.uniform(ROOM[1][0], ROOM[1][1]))
        if all(rect_dist(q, z) > 0.3 for z in zones):
            qrs[f"t{len(qrs)}"] = q
    return dict(seed=seed, zones=[tuple(round(v, 1) for v in z) for z in zones], qrs=qrs,
                lag=r.uniform(0.5, 1.3), speed=float(os.environ['STRESS_SPEED']) if os.environ.get('STRESS_SPEED') else r.choice([0.5, 1.0, 1.5]),
                map_err=(r.uniform(-0.7, 0.7), r.uniform(-0.7, 0.7)), noise=r.choice([0.0, 0.1, 0.2]))


def run(seed):
    sc = scenario(seed)
    t0 = time.time()
    out = dict(sc, ok=True, why=[])
    try:
        res = T.fly([[(z[0], z[1]), (z[2], z[1]), (z[2], z[3]), (z[0], z[3])] for z in sc["zones"]],
                    drone_speed=sc["speed"], qrs=sc["qrs"], map_err=sc["map_err"],
                    noise=sc["noise"], rng=random.Random(seed), max_steps=2_000_000,
                    speed=sc['speed'], dt=0.1, margin=float(os.environ.get('STRESS_MARGIN', 2.0)), dynamic=os.environ.get('STRESS_DYNAMIC', '1') != '0',
                    alt=float(os.environ.get('STRESS_ALT', 3.0)), lag=float(os.environ['STRESS_LAG']) if os.environ.get('STRESS_LAG') else sc['lag'])
    except AssertionError as e:
        out.update(ok=False, why=[f"did not finish: {e}"], closest=None, time=None, gave_up=None)
        return out
    out["closest"] = res["closest"]
    out["time"] = res["time"]
    out["gave_up"] = sum("given up" in m or "dropped" in m for m in res["log"])
    if res["closest"] <= 0.0:
        out["why"].append(f"ENTERED RED (closest {res['closest']:.2f} m)")
    miss = [(k, round(min(rect_dist(sc['qrs'][k], z) for z in sc['zones']), 1))
            for k in sorted(set(sc["qrs"]) - res["seen"])]
    bad = [m for m in miss if m[1] >= 2.5]
    out["missed"] = miss
    if bad:
        out["why"].append(f"missed tiles clear of red: {bad}")
    out["ok"] = not out["why"]
    out["wall"] = time.time() - t0
    return out


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    first = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    with mp.Pool() as pool:
        results = pool.map(run, range(first, first + n))
    bad = [r for r in results if not r["ok"]]
    closest = sorted(r["closest"] for r in results if r.get("closest") is not None)
    print(f"{n - len(bad)}/{n} scenarios passed;  closest approach to red: min {closest[0]:.2f} m, "
          f"median {closest[len(closest) // 2]:.2f} m")
    for r in bad:
        print(f"FAIL seed {r['seed']}: {'; '.join(r['why'])}\n     zones {r['zones']}  lag {r['lag']:.2f}  "
              f"speed {r['speed']}  map_err {tuple(round(v, 2) for v in r['map_err'])}  noise {r['noise']}")
    soft = [r for r in results if r["ok"] and r.get("missed")]
    print(f"{len(soft)} passing scenarios still missed a tile within 2.5 m of red (expected: inside the margin)")
