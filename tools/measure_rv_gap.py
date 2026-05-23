"""Measure the gap between the greedy rendezvous and a theoretical lower bound.

For a given fixture, runs the full DynamicMissionPlanner pipeline (grid
search chooses the winning angle deterministically), extracts the cut
points (drone positions at the moment of each service break), and
compares the sum of euclidean cut->rv distances produced by the greedy
planner against the sum of cut-to-nearest-polyline distances, which is
a lower bound for any rendezvous strategy on the same cut points.

Pipeline code is not modified; we only measure.
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

# (fixture path, expected winning angle from the grid-search dump runs)
FIXTURES = [
    ("tests/test_fields/dynamic/polyline_diagonal.json", 7),
    ("tests/test_fields/dynamic/field_with_obstacles.json", 75),
    ("tests/test_fields/dynamic/polyline_along_edge.json", 179),
]


def _load_field(path):
    with open(path) as f:
        data = json.load(f)
    boundary = [tuple(p) for p in data["boundary"]]
    if boundary[0] == boundary[-1]:
        boundary = boundary[:-1]
    return {
        "name": data.get("name", os.path.basename(path)),
        "boundary": boundary,
        "base_point": tuple(data["base_point"]),
        "ugv_polyline": [tuple(p) for p in data["ugv_polyline"]],
        "ugv_speed": float(data.get("ugv_speed", 2.0)),
        "ugv_t_service": float(data.get("ugv_t_service", 300.0)),
    }


def _open_db():
    return sessionmaker(bind=create_engine("sqlite:///agriswarm.db"))()


def _run_mission(field):
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
        return result
    finally:
        db.close()


def _extract_cut(cycle):
    """Return (cut_point, rv_point) for a cycle closed by a rendezvous.

    Walks segments from the end while they are deadheads; the p2 of the
    last non-deadhead segment is the cut point. Returns None if the cycle
    has no non-deadhead segments (defensive).
    """
    segs = cycle.get("segments", [])
    k = len(segs) - 1
    while k >= 0 and segs[k].get("segment_type") == "deadhead":
        k -= 1
    if k < 0:
        return None
    p2 = segs[k]["p2"]
    cut_point = (float(p2[0]), float(p2[1]))
    rv = cycle["base_point"]
    rv_point = (float(rv[0]), float(rv[1]))
    return cut_point, rv_point


def _nearest_polyline(px, py, polyline):
    """Min euclidean distance from (px, py) to the polyline (interpolated).

    Returns (best_distance, segment_index_of_foot).
    Same math as GridSearchOptimizer._pt_to_polyline_dist, but also reports
    which polyline segment the foot fell on, so operators can read how
    far along the UGV path the LB prefers the service.
    """
    best_d = float("inf")
    best_i = 0
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
        fx = ax + t * dx
        fy = ay + t * dy
        d = math.hypot(px - fx, py - fy)
        if d < best_d:
            best_d = d
            best_i = i
    return best_d, best_i


def _nearest_candidate_idx(point, candidates):
    """Index of the RV candidate closest to `point` (should be exact match)."""
    best_d = float("inf")
    best_i = 0
    for i, c in enumerate(candidates):
        p = c["point"]
        d = math.hypot(point[0] - p[0], point[1] - p[1])
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def _report_fixture(path, expected_angle):
    field = _load_field(path)
    print("=" * 80)
    print(f"Fixture: {field['name']}  |  expected angle: {expected_angle}")
    try:
        result = _run_mission(field)
    except Exception as exc:
        print(f"  PIPELINE FAILED: {exc}")
        print()
        return

    cycles = result.get("mission_cycles", [])
    actual_angle = result.get("best_angle")
    print(f"Actual winning angle: {actual_angle}  |  total cycles: {len(cycles)}")

    rv_cycles = [c for c in cycles if "rv_wait_s" in c]
    if not rv_cycles:
        print("No rendezvous cycles: mission produced a single cycle. Skipping.")
        print()
        return

    # Reinstantiate planner only to access the candidate list for
    # reporting rv_idx_greedy. This is identical to the one used inside
    # run_mission_planning (same v_ugv, t_service, candidate_spacing).
    rp = RendezvousPlanner(
        ugv_polyline=field["ugv_polyline"],
        v_ugv=field["ugv_speed"],
        t_service=field["ugv_t_service"],
    )
    candidates = rp._candidates
    polyline = field["ugv_polyline"]

    print(f"Cuts: {len(rv_cycles)} rendezvous in mission")
    print()
    print(f"  {'cut_idx':>7}  {'greedy_dist':>11}  {'lb_dist':>8}  "
          f"{'delta':>7}  {'rv_idx_greedy':>13}  {'lb_seg_idx':>10}")

    total_greedy = 0.0
    total_lb = 0.0
    for i, cycle in enumerate(rv_cycles):
        pair = _extract_cut(cycle)
        if pair is None:
            print(f"  {i:7d}  (no non-deadhead segment, skipped)")
            continue
        cut, rv = pair
        greedy = math.hypot(cut[0] - rv[0], cut[1] - rv[1])
        lb, seg_idx = _nearest_polyline(cut[0], cut[1], polyline)
        delta = greedy - lb
        rv_idx = _nearest_candidate_idx(rv, candidates)
        total_greedy += greedy
        total_lb += lb
        print(f"  {i:7d}  {greedy:11.1f}  {lb:8.1f}  {delta:7.1f}  "
              f"{rv_idx:13d}  {seg_idx:10d}")

    print()
    if total_lb > 1e-6:
        gap_pct = (total_greedy - total_lb) / total_lb * 100.0
        print(f"  Totals: greedy_dh={total_greedy:.1f}m  "
              f"lb_dh={total_lb:.1f}m  gap={gap_pct:.1f}%")
    else:
        print(f"  Totals: greedy_dh={total_greedy:.1f}m  "
              f"lb_dh={total_lb:.1f}m  (LB is ~0, gap undefined)")
    print()


def main():
    for path, expected_angle in FIXTURES:
        _report_fixture(path, expected_angle)


if __name__ == "__main__":
    main()
