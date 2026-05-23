"""Sweep the UGV-advance penalty weight (gamma) across fixtures,
holding the sweep route fixed.

Procedure:
  1. Run the full pipeline once with default gamma to obtain the winning
     angle and its route_segments.
  2. For each gamma in GAMMAS, instantiate a fresh RendezvousPlanner with
     that gamma and call plan_dynamic_cycles directly on the same route.
  3. Extract cuts, compute greedy_dh and lb_dh, report gap and tail%.

Varying only gamma (not the route) isolates whether re-weighting changes
the rendezvous choices the greedy makes.

The pipeline is not modified. Usage:
    source venv/bin/activate
    python tools/sweep_gamma.py
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from shapely.geometry import Polygon as ShapelyPolygon

from backend.models.drone_model import Drone
from backend.services.mission_planner import DynamicMissionPlanner
from backend.algorithms.rendezvous.planner import RendezvousPlanner
from backend.algorithms.energy.energy_model import DroneEnergyModel
from backend.algorithms.energy.segmentation import MissionSegmenter
from backend.algorithms.coverage.margin import MarginReducer


DRONE_NAME = "DJI Agras T30"
SWATH = 5.0

FIXTURES = [
    "tests/test_fields/dynamic/polyline_diagonal.json",
    "tests/test_fields/dynamic/field_with_obstacles.json",
    "tests/test_fields/dynamic/polyline_along_edge.json",
]

GAMMAS = [0.3, 1.0, 2.0, 5.0, 10.0]


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


def _run_grid_once(field):
    """Run the full pipeline once with default gamma and return the
    ingredients we need to re-run plan_dynamic_cycles directly."""
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
        drone = db.query(Drone).filter(Drone.name == DRONE_NAME).first()
    finally:
        db.close()
    return drone, result


def _build_segmenter(drone, energy_model):
    return MissionSegmenter(
        drone,
        target_rate_l_ha=20.0,
        work_speed_kmh=5.0 * 3.6,
        swath_width=SWATH,
        energy_model=energy_model,
    )


def _extract_cut(cycle):
    segs = cycle.get("segments", [])
    k = len(segs) - 1
    while k >= 0 and segs[k].get("segment_type") == "deadhead":
        k -= 1
    if k < 0:
        return None
    p2 = segs[k]["p2"]
    cut = (float(p2[0]), float(p2[1]))
    rv = cycle["base_point"]
    rv_point = (float(rv[0]), float(rv[1]))
    return cut, rv_point


def _nearest_polyline(px, py, polyline):
    best_d = float("inf")
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
        if d < best_d:
            best_d = d
    return best_d


def _nearest_candidate_idx(point, candidates):
    best_d = float("inf")
    best_i = 0
    for i, c in enumerate(candidates):
        p = c["point"]
        d = math.hypot(point[0] - p[0], point[1] - p[1])
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def _measure_with_gamma(field, safe_polygon, route_segments, drone, energy_model, gamma):
    rp = RendezvousPlanner(
        ugv_polyline=field["ugv_polyline"],
        v_ugv=field["ugv_speed"],
        t_service=field["ugv_t_service"],
        gamma=gamma,
    )
    segmenter = _build_segmenter(drone, energy_model)
    plan = rp.plan_dynamic_cycles(segmenter, safe_polygon, route_segments)
    cycles = plan.get("cycles", [])
    rv_cycles = [c for c in cycles if "rv_wait_s" in c]
    if not rv_cycles:
        return {"cuts": 0}

    polyline = field["ugv_polyline"]
    candidates = rp._candidates
    tail_idx = len(candidates) - 1

    total_greedy = 0.0
    total_lb = 0.0
    tail_cuts = 0
    n = 0
    for c in rv_cycles:
        pair = _extract_cut(c)
        if pair is None:
            continue
        cut, rv = pair
        total_greedy += math.hypot(cut[0] - rv[0], cut[1] - rv[1])
        total_lb += _nearest_polyline(cut[0], cut[1], polyline)
        rv_idx = _nearest_candidate_idx(rv, candidates)
        if rv_idx == tail_idx:
            tail_cuts += 1
        n += 1

    gap = (total_greedy - total_lb) / total_lb * 100.0 if total_lb > 1e-6 else float("inf")
    return {
        "cuts": n,
        "greedy": total_greedy,
        "lb": total_lb,
        "gap_pct": gap,
        "tail_pct": (tail_cuts / n * 100.0) if n else 0.0,
        "tail_idx": tail_idx,
        "n_cycles": len(cycles),
    }


def main():
    print("Phase 1: running grid search once per fixture with default gamma...")
    prepared = []
    for path in FIXTURES:
        field = _load_field(path)
        drone, result = _run_grid_once(field)
        winning_angle = result.get("best_angle")
        route_segments = result.get("route_segments", [])
        safe_polygon = result.get("safe_polygon")
        energy_model = DroneEnergyModel(drone)
        prepared.append({
            "field": field,
            "drone": drone,
            "energy_model": energy_model,
            "safe_polygon": safe_polygon,
            "route_segments": route_segments,
            "winning_angle": winning_angle,
        })
        print(f"  {field['name']}: winning angle = {winning_angle}  "
              f"route = {len(route_segments)} segments")

    print()
    print("Phase 2: sweep gamma on the same route, measure cuts...")
    print()
    header = (
        f"  {'gamma':>6}  {'fixture':<24}  {'cycles':>6}  {'cuts':>4}  "
        f"{'greedy_m':>9}  {'lb_m':>7}  {'gap%':>7}  {'tail%':>6}"
    )
    print(header)
    print("-" * len(header))
    for gamma in GAMMAS:
        for p in prepared:
            r = _measure_with_gamma(
                p["field"], p["safe_polygon"], p["route_segments"],
                p["drone"], p["energy_model"], gamma,
            )
            name = p["field"]["name"]
            if r["cuts"] == 0:
                print(f"  {gamma:>6.2f}  {name:<24}  no rendezvous cuts")
                continue
            gap_s = f"{r['gap_pct']:.1f}" if math.isfinite(r["gap_pct"]) else "inf"
            print(
                f"  {gamma:>6.2f}  {name:<24}  {r['n_cycles']:>6}  {r['cuts']:>4}  "
                f"{r['greedy']:>9.1f}  {r['lb']:>7.1f}  {gap_s:>7}  "
                f"{r['tail_pct']:>6.0f}"
            )
        print()


if __name__ == "__main__":
    main()
