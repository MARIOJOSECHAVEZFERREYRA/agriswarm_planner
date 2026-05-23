"""Batch-run grid vs GA across every field in tests/test_fields/.

For each field, runs both strategies through the real mission pipeline
and records how much (if anything) the GA gives up relative to grid.
Grid is optimal by construction (180 integer angles, exhaustive), so
the GA's gap quantifies suboptimality; the speedup quantifies the win.

Usage:
    source venv/bin/activate
    python tools/batch_grid_vs_ga.py
    python tools/batch_grid_vs_ga.py --only dynamic  # single category
    python tools/batch_grid_vs_ga.py --out report.json
"""

import argparse
import json
import os
import sys
import time
import traceback

sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from backend.services.mission_planner import (
    StaticMissionPlanner, DynamicMissionPlanner,
)


FIELDS_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), "tests", "test_fields"
)
CATEGORIES = ["basic", "organic", "stress_tests", "dynamic"]
DRONE_NAME = "DJI Agras T30"
CLOSE_DEG_THRESHOLD = 2.0


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
    metrics = result.get("metrics", {}) or {}
    cycles = result.get("mission_cycles", []) or []
    gen_stats = result.get("gen_stats", []) or []
    return {
        "angle": float(result.get("best_angle", 0.0)),
        "n_cycles": len(cycles),
        "flight_time_min": float(metrics.get("flight_time_min", 0.0)),
        "total_op_time_min": float(metrics.get("total_op_time_min", 0.0)),
        "productivity_ha_hr": float(metrics.get("productivity_ha_hr", 0.0)),
        "spray_km": float(metrics.get("spray_dist_km", 0.0)),
        "dead_km": float(metrics.get("dead_dist_km", 0.0)),
        "efficiency_pct": float(metrics.get("efficiency_ratio", 0.0)),
        "rv_n": int(metrics.get("rv_n_rendezvous", 0)),
        "rv_wait_min": float(metrics.get("rv_wait_min", 0.0)),
        "elapsed_s": elapsed,
        "n_generations": len(gen_stats),
    }


def _verdict(angle_diff):
    a = abs(angle_diff)
    if a == 0:
        return "MATCH"
    if a <= CLOSE_DEG_THRESHOLD:
        return "close"
    return "DIVERGE"


def _collect_fields(only=None):
    cats = [only] if only else CATEGORIES
    fields = []
    for cat in cats:
        d = os.path.join(FIELDS_ROOT, cat)
        if not os.path.isdir(d):
            continue
        for fname in sorted(os.listdir(d)):
            if fname.endswith(".json"):
                fields.append((cat, os.path.join(d, fname)))
    return fields


def _mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def _fmt_row(name, mode, angle_g, angle_a, gap_pct, sp, verdict):
    return (f"{name:<30} {mode:<8} {angle_g:>6.0f}° {angle_a:>6.0f}° "
            f"{gap_pct:>+7.2f}% {sp:>6.2f}x  {verdict}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--only", choices=CATEGORIES, default=None,
                        help="Restrict to one category")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    fields = _collect_fields(args.only)
    if not fields:
        print("No fields found.")
        return

    engine = create_engine("sqlite:///agriswarm.db")
    Session = sessionmaker(bind=engine)

    print(f"Evaluating {len(fields)} field(s) across grid + GA ...\n")
    header = (f"{'field':<30} {'mode':<8} {'gridθ':>7} {'gaθ':>7} "
              f"{'gap':>8} {'speed':>7}  verdict")
    print(header)
    print("-" * len(header))

    rows = []
    failures = []

    for cat, path in fields:
        field_stem = os.path.splitext(os.path.basename(path))[0]
        db = Session()
        try:
            field = _load_field(path)
            mode = "dynamic" if field["is_dynamic"] else "static"

            grid_s = _run(field, "grid", db)
            ga_s = _run(field, "genetic", db)

            gap_pct = ((ga_s["total_op_time_min"] - grid_s["total_op_time_min"])
                       / grid_s["total_op_time_min"] * 100.0) if grid_s["total_op_time_min"] else 0.0
            speedup = (grid_s["elapsed_s"] / ga_s["elapsed_s"]
                       if ga_s["elapsed_s"] > 0 else 0.0)
            angle_diff = ga_s["angle"] - grid_s["angle"]
            verdict = _verdict(angle_diff)

            row = {
                "category": cat,
                "field": field_stem,
                "mode": mode,
                "grid": grid_s,
                "ga": ga_s,
                "angle_diff_deg": angle_diff,
                "op_time_gap_pct": gap_pct,
                "speedup_x": speedup,
                "verdict": verdict,
            }
            rows.append(row)
            print(_fmt_row(field_stem, mode, grid_s["angle"],
                           ga_s["angle"], gap_pct, speedup, verdict))
        except Exception as exc:
            failures.append({"field": field_stem, "error": str(exc)})
            print(f"{field_stem:<30} ERROR: {exc}")
            traceback.print_exc()
        finally:
            db.close()

    # Aggregates
    if rows:
        gaps = [r["op_time_gap_pct"] for r in rows]
        speedups = [r["speedup_x"] for r in rows]
        angle_diffs = [abs(r["angle_diff_deg"]) for r in rows]

        n_total = len(rows)
        n_match = sum(1 for r in rows if r["verdict"] == "MATCH")
        n_close = sum(1 for r in rows if r["verdict"] == "close")
        n_diverge = sum(1 for r in rows if r["verdict"] == "DIVERGE")
        n_optimal = sum(1 for r in rows if r["op_time_gap_pct"] <= 0.01)
        worst = max(rows, key=lambda r: r["op_time_gap_pct"])

        by_mode = {"static": [], "dynamic": []}
        for r in rows:
            by_mode[r["mode"]].append(r)

        print("\n" + "=" * 64)
        print("SUMMARY")
        print("=" * 64)
        print(f"Fields evaluated:     {n_total}")
        if failures:
            print(f"Failures:             {len(failures)}")
        print(f"Angle verdict:        MATCH={n_match}  close={n_close}  DIVERGE={n_diverge}")
        print(f"GA optimal (≤0.01%):  {n_optimal} / {n_total}")
        print(f"Mean op-time gap:     {_mean(gaps):+.3f}%   (max {worst['op_time_gap_pct']:+.2f}% @ {worst['field']})")
        print(f"Median op-time gap:   {sorted(gaps)[len(gaps)//2]:+.3f}%")
        print(f"Mean |angle diff|:    {_mean(angle_diffs):.1f}°")
        print(f"Mean speedup:         {_mean(speedups):.2f}x")

        for mode, rs in by_mode.items():
            if not rs:
                continue
            gs = [r["op_time_gap_pct"] for r in rs]
            sps = [r["speedup_x"] for r in rs]
            print(f"  [{mode:<7}] n={len(rs):<3} mean gap={_mean(gs):+.3f}%  "
                  f"mean speedup={_mean(sps):.2f}x")

    report = {
        "drone": DRONE_NAME,
        "n_fields": len(rows),
        "n_failures": len(failures),
        "threshold_close_deg": CLOSE_DEG_THRESHOLD,
        "rows": rows,
        "failures": failures,
    }
    if rows:
        report["summary"] = {
            "match": n_match,
            "close": n_close,
            "diverge": n_diverge,
            "ga_optimal_count": n_optimal,
            "mean_op_time_gap_pct": _mean(gaps),
            "max_op_time_gap_pct": worst["op_time_gap_pct"],
            "max_op_time_gap_field": worst["field"],
            "mean_angle_diff_deg": _mean(angle_diffs),
            "mean_speedup_x": _mean(speedups),
        }

    if args.out is None:
        out_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                               "tests", "test_results")
        os.makedirs(out_dir, exist_ok=True)
        args.out = os.path.join(out_dir, "batch_grid_vs_ga.json")

    with open(args.out, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\nReport saved: {args.out}")


if __name__ == "__main__":
    main()
