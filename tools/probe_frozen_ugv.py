"""Probe: what happens if the UGV barely moves during drone flight?

Runs plan_dynamic_cycles twice on the same route:
  - baseline v_ugv=2.0 m/s (current model)
  - frozen   v_ugv=0.001 m/s (UGV stays put)

If the only reason cuts cluster at the tail is kinematic drift, the
frozen case should spread rv_idx across the polyline.
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
from backend.algorithms.energy.energy_model import DroneEnergyModel
from backend.algorithms.energy.segmentation import MissionSegmenter


DRONE_NAME = "DJI Agras T30"
FIELD = "tests/test_fields/dynamic/polyline_diagonal.json"


def _load_field(path):
    with open(path) as f:
        data = json.load(f)
    b = [tuple(p) for p in data["boundary"]]
    if b[0] == b[-1]:
        b = b[:-1]
    return {
        "boundary": b,
        "base_point": tuple(data["base_point"]),
        "ugv_polyline": [tuple(p) for p in data["ugv_polyline"]],
        "ugv_speed": float(data.get("ugv_speed", 2.0)),
        "ugv_t_service": float(data.get("ugv_t_service", 300.0)),
    }


def _open_db():
    return sessionmaker(bind=create_engine("sqlite:///agriswarm.db"))()


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


def _run_once(field, v_ugv, drone, result):
    rp = RendezvousPlanner(
        ugv_polyline=field["ugv_polyline"],
        v_ugv=v_ugv,
        t_service=field["ugv_t_service"],
    )
    segmenter = MissionSegmenter(
        drone, target_rate_l_ha=20.0, work_speed_kmh=5.0 * 3.6,
        swath_width=5.0, energy_model=DroneEnergyModel(drone),
    )
    plan = rp.plan_dynamic_cycles(
        segmenter,
        result.get("safe_polygon"),
        result.get("route_segments"),
    )
    return plan, rp


def _analyze(label, plan, rp, polyline):
    cycles = plan.get("cycles", [])
    rv_cycles = [c for c in cycles if "rv_wait_s" in c]
    candidates = rp._candidates
    n_cand = len(candidates)
    tail_idx = n_cand - 1

    print(f"=== {label} ===")
    print(f"cycles={len(cycles)}  rv cycles={len(rv_cycles)}  "
          f"candidate count={n_cand}  polyline_len={rp.total_length:.1f}")

    rv_idx_counts = [0] * n_cand
    greedy_total = 0.0
    lb_total = 0.0
    rows = []
    for c in rv_cycles:
        pair = _extract_cut(c)
        if pair is None:
            continue
        cut, rv = pair
        best = float("inf")
        best_i = 0
        for i, cand in enumerate(candidates):
            d = math.hypot(rv[0] - cand["point"][0], rv[1] - cand["point"][1])
            if d < best:
                best = d
                best_i = i
        rv_idx_counts[best_i] += 1
        g = math.hypot(cut[0] - rv[0], cut[1] - rv[1])
        lb = _nearest_polyline(cut[0], cut[1], polyline)
        greedy_total += g
        lb_total += lb
        rows.append((cut, rv, best_i, g, lb, candidates[best_i]["distance_along"]))

    gap = (greedy_total - lb_total) / lb_total * 100.0 if lb_total > 1e-6 else float("inf")
    tail_pct = rv_idx_counts[tail_idx] / max(1, sum(rv_idx_counts)) * 100.0

    print(f"greedy_dh={greedy_total:.1f}m  lb_dh={lb_total:.1f}m  "
          f"gap={gap:.1f}%  tail%={tail_pct:.0f}")
    print()
    print("rv_idx histogram (cand idx : cuts):")
    for i, n in enumerate(rv_idx_counts):
        if n > 0:
            bar = "#" * n
            da = candidates[i]["distance_along"]
            print(f"  idx {i:>3}  d_along={da:>6.1f}  cuts={n:>3}  {bar}")
    print()
    print(f"cuts detail ({len(rows)}):")
    print(f"  {'i':>3}  {'cut_xy':>18}  {'rv_idx':>6}  {'rv_da':>7}  "
          f"{'greedy':>7}  {'lb':>6}")
    for i, (cut, rv, rvi, g, lb, da) in enumerate(rows):
        print(f"  {i:>3}  ({cut[0]:>7.1f},{cut[1]:>7.1f})  "
              f"{rvi:>6}  {da:>7.1f}  {g:>7.1f}  {lb:>6.1f}")
    print()


def main():
    field = _load_field(FIELD)

    print("Phase 1: get route via grid search (default v_ugv)")
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
            overrides={"swath": 5.0, "speed": 5.0, "app_rate": 20.0},
            base_point=field["base_point"],
            strategy_name="grid",
        )
        drone = db.query(Drone).filter(Drone.name == DRONE_NAME).first()
    finally:
        db.close()
    print(f"winner angle={result.get('best_angle')}")
    print()

    polyline = field["ugv_polyline"]
    plan_a, rp_a = _run_once(field, 2.0, drone, result)
    _analyze("baseline v_ugv=2.0 m/s", plan_a, rp_a, polyline)
    plan_b, rp_b = _run_once(field, 0.001, drone, result)
    _analyze("frozen v_ugv=0.001 m/s", plan_b, rp_b, polyline)


if __name__ == "__main__":
    main()
