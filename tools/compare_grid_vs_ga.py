"""Compare GridSearchOptimizer vs SweepAngleOptimizer on the same field.

Runs both strategies through the REAL mission pipeline (optimizer +
MissionSegmenter + MissionAnalyzer) so numbers match what the UI shows.
Saves a JSON report with timings, winner angles, mission metrics, and
per-generation stats for tuning the GA against grid ground truth.

Usage:
    source venv/bin/activate
    python tools/compare_grid_vs_ga.py [field.json] [--out report.json]
"""

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from backend.services.mission_planner import (
    StaticMissionPlanner, DynamicMissionPlanner,
)


DEFAULT_FIELD = "tests/test_fields/dynamic/polyline_diagonal.json"
DRONE_NAME = "DJI Agras T30"


def _load_field(path):
    with open(path) as f:
        data = json.load(f)
    boundary = [tuple(p) for p in data["boundary"]]
    if boundary[0] == boundary[-1]:
        boundary = boundary[:-1]

    polyline = data.get("ugv_polyline")
    is_dynamic = polyline is not None

    obstacles = [
        [tuple(p) for p in obs] for obs in data.get("obstacles", [])
    ] or None

    return {
        "name": data.get("name", os.path.basename(path)),
        "path": path,
        "boundary": boundary,
        "base_point": tuple(data["base_point"]),
        "is_dynamic": is_dynamic,
        "ugv_polyline": [tuple(p) for p in polyline] if is_dynamic else None,
        "ugv_speed": float(data.get("ugv_speed", 2.0)),
        "ugv_t_service": float(data.get("ugv_t_service", 300.0)),
        "obstacles": obstacles,
    }


def _build_planner(field):
    if field["is_dynamic"]:
        return DynamicMissionPlanner(
            ugv_polyline=field["ugv_polyline"],
            ugv_speed=field["ugv_speed"],
            ugv_t_service=field["ugv_t_service"],
        )
    return StaticMissionPlanner()


def _run(field, strategy_name, db):
    planner = _build_planner(field)
    t0 = time.perf_counter()
    result = planner.run_mission_planning(
        db=db,
        polygon_points=field["boundary"],
        drone_name=DRONE_NAME,
        overrides={},
        base_point=field["base_point"],
        strategy_name=strategy_name,
        obstacle_polygons=field["obstacles"],
    )
    elapsed = time.perf_counter() - t0
    return result, elapsed


def _summarize(result, elapsed_s):
    metrics = result.get("metrics", {}) or {}
    cycles = result.get("mission_cycles", []) or []
    gen_stats = result.get("gen_stats", []) or []
    return {
        "angle": float(result.get("best_angle", 0.0)),
        "n_cycles": len(cycles),
        "area_ha": float(metrics.get("area_ha", 0.0)),
        "flight_time_min": float(metrics.get("flight_time_min", 0.0)),
        "total_op_time_min": float(metrics.get("total_op_time_min", 0.0)),
        "productivity_ha_hr": float(metrics.get("productivity_ha_hr", 0.0)),
        "spray_km": float(metrics.get("spray_dist_km", 0.0)),
        "ferry_km": float(metrics.get("ferry_dist_km", 0.0)),
        "deadhead_km": float(metrics.get("deadhead_dist_km", 0.0)),
        "dead_km": float(metrics.get("dead_dist_km", 0.0)),
        "efficiency_pct": float(metrics.get("efficiency_ratio", 0.0)),
        "rv_n": int(metrics.get("rv_n_rendezvous", 0)),
        "rv_wait_min": float(metrics.get("rv_wait_min", 0.0)),
        "elapsed_s": elapsed_s,
        "n_generations": len(gen_stats),
        "gen_stats": gen_stats,
    }


def _delta(ga, grid):
    def pct(a, b):
        return (a - b) / b * 100.0 if b else 0.0
    return {
        "angle_diff_deg": ga["angle"] - grid["angle"],
        "op_time_pct": pct(ga["total_op_time_min"], grid["total_op_time_min"]),
        "op_time_diff_min": ga["total_op_time_min"] - grid["total_op_time_min"],
        "cycles_diff": ga["n_cycles"] - grid["n_cycles"],
        "speedup_x": grid["elapsed_s"] / ga["elapsed_s"] if ga["elapsed_s"] > 0 else 0.0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("field", nargs="?", default=DEFAULT_FIELD)
    parser.add_argument("--out", default=None,
                        help="Output JSON path (default: tests/test_results/grid_vs_ga_<field>.json)")
    args = parser.parse_args()

    field = _load_field(args.field)

    engine = create_engine("sqlite:///agriswarm.db")
    Session = sessionmaker(bind=engine)
    db = Session()

    try:
        mode = "dynamic" if field["is_dynamic"] else "static"
        print(f"Field: {field['name']}  |  mode={mode}  |  drone={DRONE_NAME}")
        print("=" * 60)

        print("\n[1/2] Running strategy=grid ...")
        grid_res, grid_elapsed = _run(field, "grid", db)
        print(f">> Grid elapsed: {grid_elapsed:.2f} s")
        grid_sum = _summarize(grid_res, grid_elapsed)

        print("\n[2/2] Running strategy=genetic ...")
        ga_res, ga_elapsed = _run(field, "genetic", db)
        print(f">> GA elapsed: {ga_elapsed:.2f} s")
        ga_sum = _summarize(ga_res, ga_elapsed)
    finally:
        db.close()

    delta = _delta(ga_sum, grid_sum)

    report = {
        "field": {
            "name": field["name"],
            "path": field["path"],
            "mode": mode,
            "drone": DRONE_NAME,
            "ugv_speed_ms": field["ugv_speed"] if field["is_dynamic"] else None,
            "ugv_t_service_s": field["ugv_t_service"] if field["is_dynamic"] else None,
        },
        "grid": grid_sum,
        "ga": ga_sum,
        "delta_ga_vs_grid": delta,
    }

    # Table
    print("\n" + "=" * 64)
    print(f"{'metric':<22} {'grid':>13} {'ga':>13} {'Δ':>13}")
    print("-" * 64)
    rows = [
        ("angle (deg)", grid_sum["angle"], ga_sum["angle"], delta["angle_diff_deg"]),
        ("op time (min)", grid_sum["total_op_time_min"], ga_sum["total_op_time_min"], delta["op_time_diff_min"]),
        ("flight time (min)", grid_sum["flight_time_min"], ga_sum["flight_time_min"], ga_sum["flight_time_min"] - grid_sum["flight_time_min"]),
        ("productivity ha/hr", grid_sum["productivity_ha_hr"], ga_sum["productivity_ha_hr"], ga_sum["productivity_ha_hr"] - grid_sum["productivity_ha_hr"]),
        ("cycles", grid_sum["n_cycles"], ga_sum["n_cycles"], delta["cycles_diff"]),
        ("spray (km)", grid_sum["spray_km"], ga_sum["spray_km"], ga_sum["spray_km"] - grid_sum["spray_km"]),
        ("dead (km)", grid_sum["dead_km"], ga_sum["dead_km"], ga_sum["dead_km"] - grid_sum["dead_km"]),
        ("efficiency %", grid_sum["efficiency_pct"], ga_sum["efficiency_pct"], ga_sum["efficiency_pct"] - grid_sum["efficiency_pct"]),
        ("rv stops", grid_sum["rv_n"], ga_sum["rv_n"], ga_sum["rv_n"] - grid_sum["rv_n"]),
        ("rv wait (min)", grid_sum["rv_wait_min"], ga_sum["rv_wait_min"], ga_sum["rv_wait_min"] - grid_sum["rv_wait_min"]),
        ("elapsed (s)", grid_sum["elapsed_s"], ga_sum["elapsed_s"], f"{delta['speedup_x']:.2f}x"),
        ("generations", grid_sum["n_generations"], ga_sum["n_generations"], ""),
    ]
    for name, g, a, d in rows:
        gv = f"{g:.2f}" if isinstance(g, float) else f"{g}"
        av = f"{a:.2f}" if isinstance(a, float) else f"{a}"
        dv = f"{d:+.2f}" if isinstance(d, (int, float)) else str(d)
        print(f"{name:<22} {gv:>13} {av:>13} {dv:>13}")

    verdict = "MATCH" if delta["angle_diff_deg"] == 0 else (
        "close" if abs(delta["angle_diff_deg"]) <= 2 else "DIVERGE")
    print(f"\nVerdict: {verdict}  |  op time gap: {delta['op_time_pct']:+.2f}%")

    if args.out is None:
        field_stem = os.path.splitext(os.path.basename(field["path"]))[0]
        out_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                               "tests", "test_results")
        os.makedirs(out_dir, exist_ok=True)
        args.out = os.path.join(out_dir, f"grid_vs_ga_{field_stem}.json")

    with open(args.out, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\nReport saved: {args.out}")


if __name__ == "__main__":
    main()
