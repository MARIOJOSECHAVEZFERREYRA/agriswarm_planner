"""Dump per-angle metrics from GridSearchOptimizer for diagnosis.

Runs the grid search pipeline for every integer angle in [0, 180) on a
representative dynamic mission and prints a table with every component
that feeds into the ranking.  The goal is to see, per angle, which term
(flight, service, alignment) dominates the fitness.

Usage:
    source venv/bin/activate
    python tools/dump_grid_angles.py
"""

import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from shapely.geometry import Polygon as ShapelyPolygon

from backend.models.drone_model import Drone
from backend.algorithms.energy.energy_model import DroneEnergyModel
from backend.algorithms.rendezvous.planner import RendezvousPlanner
from backend.algorithms.coverage.margin import MarginReducer
from backend.algorithms.coverage.path_planner import BoustrophedonPlanner
from backend.algorithms.routing.grid_search_optimizer import GridSearchOptimizer
from backend.algorithms.routing.sweep_angle_optimizer import build_obstacle_union


DEFAULT_FIELD = "tests/test_fields/dynamic/polyline_diagonal.json"
DRONE_NAME = "DJI Agras T30"
SWATH = 5.0


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


def _drone_and_model():
    engine = create_engine("sqlite:///agriswarm.db")
    db = sessionmaker(bind=engine)()
    try:
        d = db.query(Drone).filter(Drone.name == DRONE_NAME).first()
        return d, DroneEnergyModel(d)
    finally:
        db.close()


def main():
    field_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_FIELD
    field = _load_field(field_path)
    drone, em = _drone_and_model()
    rp = RendezvousPlanner(
        ugv_polyline=field["ugv_polyline"],
        v_ugv=field["ugv_speed"],
        t_service=field["ugv_t_service"],
    )
    polygon = ShapelyPolygon(field["boundary"])
    safe = MarginReducer.shrink(polygon, margin_h=SWATH / 2.0)
    obstacle_union = build_obstacle_union(safe)

    boustro = BoustrophedonPlanner(spray_width=SWATH)
    gs = GridSearchOptimizer(
        planner=boustro, angle_step=1,
        energy_model=em, rendezvous_planner=rp, w_rv=0.5,
    )

    t_service = rp.t_service
    v_uav = float(drone.speed_max_ms)

    print(f"Field: {field['name']}")
    print(f"Polyline length: {rp.total_length:.1f} m  |  v_ugv={rp.v_ugv} m/s  |"
          f"  t_service={t_service} s  |  v_uav={v_uav} m/s  |  w_rv={gs.w_rv}")
    print(f"Service discount factor: {GridSearchOptimizer._SERVICE_DISCOUNT}")
    print()

    header = (
        "  ang feas   cyc    rv  wait_s   spray_m  ferry_m  flight_only"
        "  svc_disc  align_s  avg_end_m    fit_l(s)"
    )
    print(header)
    print("-" * len(header))

    rows = []
    for a in range(0, 180, 1):
        r = gs._evaluate_angle(a, safe, safe.area, obstacle_union, field["base_point"])
        if r is None:
            print(f"  {a:3d}  (no route)")
            continue
        feas = r.get("rv_feasible", False)
        cyc = r.get("n_cycles", 0)
        rv_count = r.get("rv_count", 0)
        rv_wait = r.get("rv_wait", 0.0)
        rv_time = r.get("rv_time", 0.0)
        spray_m = r.get("spray_m", 0.0)
        ferry_m = r.get("ferry_m", 0.0)
        l = r.get("l", float("inf"))

        if feas:
            flight_only = rv_time - rv_count * t_service
            svc_disc = rv_count * t_service * GridSearchOptimizer._SERVICE_DISCOUNT
            avg_end = gs._avg_min_endpoint_polyline_dist(r.get("route_segments", []))
            align_s = gs.w_rv * rv_count * avg_end / v_uav
        else:
            flight_only = svc_disc = align_s = avg_end = float("nan")

        row = (a, feas, cyc, rv_count, rv_wait, spray_m, ferry_m,
               flight_only, svc_disc, align_s, avg_end, l)
        rows.append(row)
        fstr = "Y" if feas else "N"
        print(f"  {a:3d}   {fstr}   {cyc:3d}   {rv_count:3d}  {rv_wait:6.1f}  "
              f"{spray_m:8.0f}  {ferry_m:7.0f}  {flight_only:11.1f}  "
              f"{svc_disc:8.1f}  {align_s:7.1f}  {avg_end:8.1f}  {l:10.1f}")

    feasibles = [r for r in rows if r[1]]
    if feasibles:
        best = min(feasibles, key=lambda x: x[11])
        print()
        print(f"Winner: angle={best[0]}  fit_l={best[11]:.1f}s  "
              f"(flight_only={best[7]:.1f}, svc_disc={best[8]:.1f}, "
              f"align={best[9]:.1f}, avg_end={best[10]:.1f}m)")
        print(f"Feasible angles: {len(feasibles)} / {len(rows)}  |  "
              f"infeasible: {len(rows) - len(feasibles)}")


if __name__ == "__main__":
    main()
