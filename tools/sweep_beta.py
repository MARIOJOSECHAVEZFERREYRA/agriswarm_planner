"""Sweep the UAV-deadhead penalty weight (beta) across fixtures under B2a.

Hypothesis under test: the greedy parks the UGV because per-metre the
UGV is 3x more expensive than the drone (gamma/v_ugv vs beta/v_uav).
Raising beta should invert that ratio and force the greedy to prefer
candidates the UGV can advance to, trading drone flight for UGV travel.
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from backend.models.drone_model import Drone
from backend.services.mission_planner import DynamicMissionPlanner
from backend.algorithms.rendezvous.planner import RendezvousPlanner


DRONE_NAME = "DJI Agras T30"
SWATH = 5.0

FIXTURES = [
    "tests/test_fields/dynamic/polyline_diagonal.json",
    "tests/test_fields/dynamic/field_with_obstacles.json",
    "tests/test_fields/dynamic/polyline_along_edge.json",
]

BETAS = [0.5, 1.0, 2.0, 5.0]


def _load_field(path):
    with open(path) as f:
        data = json.load(f)
    b = [tuple(p) for p in data["boundary"]]
    if b[0] == b[-1]:
        b = b[:-1]
    return {
        "name": data.get("name", os.path.basename(path)),
        "boundary": b,
        "base_point": tuple(data["base_point"]),
        "ugv_polyline": [tuple(p) for p in data["ugv_polyline"]],
        "ugv_speed": float(data.get("ugv_speed", 2.0)),
        "ugv_t_service": float(data.get("ugv_t_service", 300.0)),
    }


def _open_db():
    return sessionmaker(bind=create_engine("sqlite:///agriswarm.db"))()


def _set_beta(beta_value):
    RendezvousPlanner.__init__.__defaults__ = (1.0, float(beta_value), 0.3, 50.0)


def _extract_cut(cycle):
    segs = cycle.get("segments", [])
    k = len(segs) - 1
    while k >= 0 and segs[k].get("segment_type") == "deadhead":
        k -= 1
    if k < 0:
        return None
    p2 = segs[k]["p2"]
    rv = cycle["base_point"]
    return ((float(p2[0]), float(p2[1])), (float(rv[0]), float(rv[1])))


def _nearest_polyline(px, py, polyline):
    best = float("inf")
    for i in range(len(polyline) - 1):
        ax, ay = polyline[i]
        bx, by = polyline[i + 1]
        dx = bx - ax
        dy = by - ay
        sl2 = dx * dx + dy * dy
        if sl2 < 1e-18:
            continue
        t = ((px - ax) * dx + (py - ay) * dy) / sl2
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
        if d < best:
            best = d
    return best


def _nearest_candidate_idx(point, candidates):
    best = float("inf")
    best_i = 0
    for i, c in enumerate(candidates):
        p = c["point"]
        d = math.hypot(point[0] - p[0], point[1] - p[1])
        if d < best:
            best = d
            best_i = i
    return best_i


def _measure(field):
    planner = DynamicMissionPlanner(
        ugv_polyline=field["ugv_polyline"],
        ugv_speed=field["ugv_speed"],
        ugv_t_service=field["ugv_t_service"],
    )
    db = _open_db()
    try:
        result = planner.run_mission_planning(
            db=db,
            polygon_points=field["boundary"],
            drone_name=DRONE_NAME,
            overrides={"swath": SWATH, "speed": 5.0, "app_rate": 20.0},
            base_point=field["base_point"],
            strategy_name="grid",
        )
    finally:
        db.close()
    angle = result.get("best_angle")
    cycles = result.get("mission_cycles", [])
    rv_cycles = [c for c in cycles if "rv_wait_s" in c]

    rp = RendezvousPlanner(
        ugv_polyline=field["ugv_polyline"],
        v_ugv=field["ugv_speed"],
        t_service=field["ugv_t_service"],
    )
    candidates = rp._candidates
    tail_idx = len(candidates) - 1
    polyline = field["ugv_polyline"]

    total_greedy = 0.0
    total_lb = 0.0
    tail_cuts = 0
    idxs_used = set()
    n = 0
    for c in rv_cycles:
        pair = _extract_cut(c)
        if pair is None:
            continue
        cut, rv = pair
        total_greedy += math.hypot(cut[0] - rv[0], cut[1] - rv[1])
        total_lb += _nearest_polyline(cut[0], cut[1], polyline)
        rvi = _nearest_candidate_idx(rv, candidates)
        idxs_used.add(rvi)
        if rvi == tail_idx:
            tail_cuts += 1
        n += 1

    gap = (total_greedy - total_lb) / total_lb * 100.0 if total_lb > 1e-6 else float("inf")
    tail_pct = (tail_cuts / n * 100.0) if n else 0.0
    return {
        "angle": angle, "cuts": n, "greedy": total_greedy, "lb": total_lb,
        "gap_pct": gap, "tail_pct": tail_pct, "unique_idxs": len(idxs_used),
    }


def main():
    header = (
        f"  {'beta':>5}  {'fixture':<24}  {'ang':>4}  {'cuts':>4}  "
        f"{'greedy_m':>9}  {'lb_m':>7}  {'gap%':>7}  {'tail%':>6}  "
        f"{'unique_idx':>10}"
    )
    print(header)
    print("-" * len(header))
    for beta in BETAS:
        _set_beta(beta)
        for path in FIXTURES:
            field = _load_field(path)
            r = _measure(field)
            name = field["name"]
            gap_s = f"{r['gap_pct']:.1f}" if math.isfinite(r["gap_pct"]) else "inf"
            print(
                f"  {beta:>5.2f}  {name:<24}  "
                f"{r['angle']:>4.0f}  {r['cuts']:>4}  "
                f"{r['greedy']:>9.1f}  {r['lb']:>7.1f}  "
                f"{gap_s:>7}  {r['tail_pct']:>6.0f}  {r['unique_idxs']:>10}"
            )
        print()


if __name__ == "__main__":
    main()
