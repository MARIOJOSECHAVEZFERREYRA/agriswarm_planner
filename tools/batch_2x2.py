"""2x2 factorial benchmark: optimization (heuristic/GA) x UGV mode (static/dynamic).

For each field, runs four scenarios through the real mission pipeline and
saves a JSON report with per-scenario metrics plus derived effects
(optimization-only, dynamic-only, combined, interaction).

Scenarios:
    S1: heuristic angle (longest-edge) + static UGV
    S2: heuristic angle (longest-edge) + dynamic UGV
    S3: GA-optimized angle + static UGV
    S4: GA-optimized angle + dynamic UGV

Usage:
    source venv/bin/activate
    python tools/batch_2x2.py                  # all fields
    python tools/batch_2x2.py --only dynamic   # one category
    python tools/batch_2x2.py --out report.json
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time
import traceback

sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from shapely.geometry import Polygon as ShapelyPolygon

from backend.services.mission_planner import (
    StaticMissionPlanner, DynamicMissionPlanner,
)


FIELDS_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), "tests", "test_fields"
)
CATEGORIES = ["basic", "organic", "stress_tests", "dynamic", "ftw"]
DRONE_NAME = "DJI Agras T30"


def _load_field(path):
    with open(path) as f:
        data = json.load(f)
    boundary = [tuple(p) for p in data["boundary"]]
    if boundary[0] == boundary[-1]:
        boundary = boundary[:-1]
    polyline = data.get("ugv_polyline")
    is_dynamic_field = polyline is not None
    obstacles = [
        [tuple(p) for p in obs] for obs in data.get("obstacles", [])
    ] or None
    return {
        "name": data.get("name", os.path.basename(path)),
        "path": path,
        "boundary": boundary,
        "base_point": tuple(data["base_point"]),
        "is_dynamic_field": is_dynamic_field,
        "ugv_polyline": [tuple(p) for p in polyline] if is_dynamic_field else None,
        "ugv_speed": float(data.get("ugv_speed", 2.0)),
        "ugv_t_service": float(data.get("ugv_t_service", 300.0)),
        "obstacles": obstacles,
    }


def _longest_edge_angle_deg(boundary):
    """Return the angle [0, 180) PERPENDICULAR to the polygon's longest edge.

    Naive baseline used in coverage-path-planning literature: sweeping
    perpendicular to the longest edge produces many short sweeps and high
    deadhead, providing a weak-but-defensible reference against which the
    optimizer's benefit is measured.
    """
    longest_len = 0.0
    long_edge_angle = 0.0
    n = len(boundary)
    for i in range(n):
        x1, y1 = boundary[i]
        x2, y2 = boundary[(i + 1) % n]
        length = math.hypot(x2 - x1, y2 - y1)
        if length > longest_len:
            longest_len = length
            long_edge_angle = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0
    return (long_edge_angle + 90.0) % 180.0


def _build_planner(field, mode):
    """mode: 'static' or 'dynamic'. Falls back to static if field has no polyline."""
    if mode == "dynamic" and field["is_dynamic_field"]:
        return DynamicMissionPlanner(
            ugv_polyline=field["ugv_polyline"],
            ugv_speed=field["ugv_speed"],
            ugv_t_service=field["ugv_t_service"],
        )
    return StaticMissionPlanner()


def _run(field, db, mode, strategy_name, strategy_params=None):
    planner = _build_planner(field, mode)
    t0 = time.perf_counter()
    result = planner.run_mission_planning(
        db=db,
        polygon_points=field["boundary"],
        drone_name=DRONE_NAME,
        overrides={},
        base_point=field["base_point"],
        strategy_name=strategy_name,
        obstacle_polygons=field["obstacles"],
        strategy_params=strategy_params or {},
    )
    elapsed = time.perf_counter() - t0
    metrics = result.get("metrics", {}) or {}
    cycles = result.get("mission_cycles", []) or []
    return {
        "angle": float(result.get("best_angle", 0.0)),
        "n_cycles": len(cycles),
        "area_ha": float(metrics.get("area_ha", 0.0)),
        "spray_km": float(metrics.get("spray_dist_km", 0.0)),
        "dead_km": float(metrics.get("dead_dist_km", 0.0)),
        "deadhead_km": float(metrics.get("deadhead_dist_km", 0.0)),
        "ferry_km": float(metrics.get("ferry_dist_km", 0.0)),
        "energy_total_wh": float(metrics.get("energy_total_wh", 0.0)),
        "energy_spray_wh": float(metrics.get("energy_spray_wh", 0.0)),
        "energy_deadhead_wh": float(metrics.get("energy_deadhead_wh", 0.0)),
        "energy_ferry_wh": float(metrics.get("energy_ferry_wh", 0.0)),
        "energy_nonprod_wh": (
            float(metrics.get("energy_deadhead_wh", 0.0))
            + float(metrics.get("energy_ferry_wh", 0.0))
        ),
        "efficiency_pct": float(metrics.get("efficiency_ratio", 0.0)),
        "rv_n": int(metrics.get("rv_n_rendezvous", 0)),
        "rv_wait_min": float(metrics.get("rv_wait_min", 0.0)),
        "elapsed_s": elapsed,
    }


def _effect_pct(before, after):
    if before <= 0:
        return 0.0
    return (before - after) / before * 100.0


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


def _resolve_single_field(spec):
    """Locate a single field JSON.

    Accepts: absolute path, path relative to CWD, relative to
    tests/test_fields/, or a bare stem (searches every category).
    """
    candidates = []
    if os.path.isabs(spec) and os.path.isfile(spec):
        candidates.append(spec)
    if os.path.isfile(spec):
        candidates.append(os.path.abspath(spec))
    rel = os.path.join(FIELDS_ROOT, spec)
    if os.path.isfile(rel):
        candidates.append(rel)
    stem = spec if spec.endswith(".json") else spec + ".json"
    # Search every subdirectory of tests/test_fields/ so --field works for
    # categories not listed in CATEGORIES (e.g. ftw).
    if os.path.isdir(FIELDS_ROOT):
        for entry in sorted(os.listdir(FIELDS_ROOT)):
            p = os.path.join(FIELDS_ROOT, entry, stem)
            if os.path.isfile(p):
                candidates.append(p)
    if not candidates:
        print(f"Field not found: {spec}")
        return []
    path = candidates[0]
    rel_to_root = os.path.relpath(path, FIELDS_ROOT)
    cat = rel_to_root.split(os.sep)[0] if os.sep in rel_to_root else "single"
    return [(cat, path)]


def _mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def _median(xs):
    return statistics.median(xs) if xs else 0.0


def _stdev(xs):
    return statistics.stdev(xs) if len(xs) >= 2 else 0.0


def _stats(xs):
    """Return mean, median, stdev, min, max as a dict."""
    return {
        "mean": _mean(xs),
        "median": _median(xs),
        "stdev": _stdev(xs),
        "min": min(xs) if xs else 0.0,
        "max": max(xs) if xs else 0.0,
        "n": len(xs),
    }


def _print_field_table(name, heur_angle, grid_angle, s1, s2, s3, s4, is_dyn,
                       eff_dead, eff_energy):
    """Pretty per-field table with the 4 scenarios and effects."""
    def _fmt(scn, key, d):
        return f"{scn[key]:.{d}f}" if scn else "n/a"

    title = f" {name}  (эвристика θ={heur_angle:.1f}°, ГА θ={grid_angle:.0f}°) "
    print("\n" + "=" * max(60, len(title)))
    print(title)
    print("=" * max(60, len(title)))
    print(f"{'Сценарий':<32} {'dead_km':>10} {'energy_wh':>12}")
    print("-" * 60)
    print(f"{'S1 (эвристика + статический)':<32} "
          f"{_fmt(s1,'dead_km',2):>10} {_fmt(s1,'energy_nonprod_wh',0):>12}")
    print(f"{'S2 (эвристика + динамический)':<32} "
          f"{_fmt(s2,'dead_km',2):>10} {_fmt(s2,'energy_nonprod_wh',0):>12}")
    print(f"{'S3 (ГА + статический)':<32} "
          f"{_fmt(s3,'dead_km',2):>10} {_fmt(s3,'energy_nonprod_wh',0):>12}")
    print(f"{'S4 (ГА + динамический)':<32} "
          f"{_fmt(s4,'dead_km',2):>10} {_fmt(s4,'energy_nonprod_wh',0):>12}")
    print("-" * 60)
    print(f"{'Эффект':<32} {'dead_km':>10} {'energy':>12}")
    print(f"{'  Оптимизация угла (S1->S3)':<32} "
          f"{eff_dead['optimization_static_pct']:>+9.2f}% "
          f"{eff_energy['optimization_static_pct']:>+11.2f}%")
    if is_dyn:
        print(f"{'  Динамическое НТС (S1->S2)':<32} "
              f"{eff_dead['dynamic_heuristic_pct']:>+9.2f}% "
              f"{eff_energy['dynamic_heuristic_pct']:>+11.2f}%")
        print(f"{'  Оптимизация на динамич.(S2->S4)':<32} "
              f"{eff_dead['optimization_dynamic_pct']:>+9.2f}% "
              f"{eff_energy['optimization_dynamic_pct']:>+11.2f}%")
        print(f"{'  Совместно (S1->S4)':<32} "
              f"{eff_dead['combined_pct']:>+9.2f}% "
              f"{eff_energy['combined_pct']:>+11.2f}%")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--only", choices=CATEGORIES, default=None,
                        help="Restrict to one category directory")
    parser.add_argument("--field", default=None,
                        help="Run a single field (path or relative to tests/test_fields/)")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    if args.field:
        fields = _resolve_single_field(args.field)
    else:
        fields = _collect_fields(args.only)
    if not fields:
        print("No fields found.")
        return

    engine = create_engine("sqlite:///agriswarm.db")
    Session = sessionmaker(bind=engine)

    print(f"Running 2x2 factorial benchmark over {len(fields)} field(s) ...\n")

    rows = []
    failures = []

    for cat, path in fields:
        field_stem = os.path.splitext(os.path.basename(path))[0]
        db = Session()
        try:
            field = _load_field(path)
            heur_angle = _longest_edge_angle_deg(field["boundary"])
            is_dyn = field["is_dynamic_field"]

            s1 = _run(field, db, "static",  "fixed",   {"angle": heur_angle})
            s2 = _run(field, db, "dynamic", "fixed",   {"angle": heur_angle}) if is_dyn else None
            s3 = _run(field, db, "static",  "grid")
            s4 = _run(field, db, "dynamic", "grid") if is_dyn else None

            row = {
                "category": cat,
                "field": field_stem,
                "is_dynamic_field": is_dyn,
                "heuristic_angle_deg": heur_angle,
                "grid_angle_deg": s3["angle"],
                "scenarios": {
                    "S1_heur_static":  s1,
                    "S2_heur_dynamic": s2,
                    "S3_opt_static":   s3,
                    "S4_opt_dynamic":  s4,
                },
            }

            # Derived effects on two primary metrics: dead_km and energy_total_wh
            def _effects(metric_key):
                v1 = s1[metric_key]
                v3 = s3[metric_key]
                out = {"optimization_static_pct": _effect_pct(v1, v3)}
                if is_dyn:
                    v2 = s2[metric_key]
                    v4 = s4[metric_key]
                    dyn_h = _effect_pct(v1, v2)
                    opt_d = _effect_pct(v2, v4)
                    comb = _effect_pct(v1, v4)
                    out.update({
                        "dynamic_heuristic_pct":    dyn_h,
                        "optimization_dynamic_pct": opt_d,
                        "combined_pct":             comb,
                        "expected_sum_pct":         opt_d + dyn_h,
                        "interaction_pct":          comb - (opt_d + dyn_h),
                    })
                return out

            row["effects_dead_km"] = _effects("dead_km")
            row["effects_energy_wh"] = _effects("energy_nonprod_wh")

            rows.append(row)

            _print_field_table(field_stem, heur_angle, s3["angle"],
                               s1, s2, s3, s4, is_dyn,
                               row["effects_dead_km"],
                               row["effects_energy_wh"])
        except Exception as exc:
            failures.append({"field": field_stem, "error": str(exc)})
            print(f"{field_stem:<30}  ERROR: {exc}")
            traceback.print_exc()
        finally:
            db.close()

    # Aggregates
    summary = {}
    if rows:
        dyn_rows = [r for r in rows if r["is_dynamic_field"]]
        summary["n_fields"] = len(rows)
        summary["n_dynamic_fields"] = len(dyn_rows)

        def _agg(metric_key, rs=None):
            """Aggregate effects stats over a (sub)set of rows."""
            rs = rows if rs is None else rs
            rs_dyn = [r for r in rs if r["is_dynamic_field"]]
            eff_key = f"effects_{metric_key}"
            out = {
                "optimization_static": _stats(
                    [r[eff_key]["optimization_static_pct"] for r in rs]),
            }
            if rs_dyn:
                out.update({
                    "optimization_dynamic": _stats(
                        [r[eff_key]["optimization_dynamic_pct"] for r in rs_dyn]),
                    "dynamic_heuristic": _stats(
                        [r[eff_key]["dynamic_heuristic_pct"] for r in rs_dyn]),
                    "combined": _stats(
                        [r[eff_key]["combined_pct"] for r in rs_dyn]),
                    "interaction": _stats(
                        [r[eff_key]["interaction_pct"] for r in rs_dyn]),
                    "n_positive_interactions": sum(
                        1 for r in rs_dyn if r[eff_key]["interaction_pct"] > 0),
                    "n_combined_improved": sum(
                        1 for r in rs_dyn if r[eff_key]["combined_pct"] > 0),
                    "n_dynamic": len(rs_dyn),
                })
            return out

        summary["global"] = {
            "dead_km": _agg("dead_km"),
            "energy_wh": _agg("energy_wh"),
        }

        # Per-category aggregates
        by_cat = {}
        for r in rows:
            by_cat.setdefault(r["category"], []).append(r)
        summary["by_category"] = {}
        for cat, rs in by_cat.items():
            summary["by_category"][cat] = {
                "n_fields": len(rs),
                "n_dynamic_fields": sum(1 for r in rs if r["is_dynamic_field"]),
                "dead_km": _agg("dead_km", rs),
                "energy_wh": _agg("energy_wh", rs),
            }

        def _line(label, st, show_n=False):
            """label: descriptive text; st: stats dict. Positive % = reduction."""
            pos = "+" if st["median"] >= 0 else ""
            tail = f"  n={st['n']}" if show_n else ""
            return (f"  {label:<36} {pos}{st['median']:5.1f}%  "
                    f"(range {st['min']:+.0f}% to {st['max']:+.0f}%){tail}")

        def _print_metric(title, agg, n_dyn):
            print(f"\n{title}   (positive % = reduction)")
            print("  " + "─" * 60)
            if n_dyn > 0 and "combined" in agg:
                comb = agg["combined"]
                imp = agg.get("n_combined_improved", 0)
                print(f"  HEADLINE — combined reduction (S1 baseline → S4 combined):")
                print(_line("    both factors together", comb))
                print(f"    → improved in {imp}/{comb['n']} fields")
                print()
                print(f"  Breakdown (contribution of each factor):")
                print(_line("    UGV dynamic alone", agg["dynamic_heuristic"]))
                print(_line("    optimization alone", agg["optimization_static"]))
                print(_line("    optimization on top of dynamic", agg["optimization_dynamic"]))
            else:
                print(_line("  optimization effect (S1 → S3)",
                            agg["optimization_static"], show_n=True))

        print("\n" + "=" * 78)
        print(f"RESULTS SUMMARY  —  {summary['n_fields']} fields ({summary['n_dynamic_fields']} dynamic)")
        print("=" * 78)
        _print_metric("[1] DEADHEAD (km)",
                      summary["global"]["dead_km"], summary["n_dynamic_fields"])
        _print_metric("[2] NON-PRODUCTIVE ENERGY (Wh)",
                      summary["global"]["energy_wh"], summary["n_dynamic_fields"])

        # Per-category breakdown
        for cat in sorted(summary["by_category"].keys()):
            cat_agg = summary["by_category"][cat]
            print("\n" + "=" * 78)
            print(f"CATEGORY: {cat}  (fields={cat_agg['n_fields']}, "
                  f"dynamic={cat_agg['n_dynamic_fields']})")
            print("=" * 78)
            _print_metric("Deadhead distance (dead_km)",
                          cat_agg["dead_km"], cat_agg["n_dynamic_fields"])
            _print_metric("Non-productive energy (deadhead + ferry, Wh)",
                          cat_agg["energy_wh"], cat_agg["n_dynamic_fields"])

    report = {
        "config": {
            "drone": DRONE_NAME,
            "optimizer": "grid",
            "angle_step": 1,
        },
        "summary": summary,
        "rows": rows,
        "failures": failures,
    }

    if args.out is None:
        out_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                               "tests", "test_results")
        os.makedirs(out_dir, exist_ok=True)
        args.out = os.path.join(out_dir, "batch_2x2.json")

    with open(args.out, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\nReport saved: {args.out}")

    # CSV export alongside the JSON (one row per field)
    csv_path = os.path.splitext(args.out)[0] + ".csv"
    _write_csv(csv_path, rows)
    print(f"CSV saved:    {csv_path}")


def _write_csv(path, rows):
    """One row per field with scenarios and effects flattened for LaTeX tables."""
    if not rows:
        return
    fieldnames = [
        "category", "field", "is_dynamic",
        "heuristic_angle", "grid_angle",
        "S1_dead_km", "S2_dead_km", "S3_dead_km", "S4_dead_km",
        "S1_energy_wh", "S2_energy_wh", "S3_energy_wh", "S4_energy_wh",
        "dead_opt_static_pct", "dead_dyn_heur_pct",
        "dead_opt_dyn_pct", "dead_combined_pct", "dead_interaction_pct",
        "energy_opt_static_pct", "energy_dyn_heur_pct",
        "energy_opt_dyn_pct", "energy_combined_pct", "energy_interaction_pct",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            s = r["scenarios"]
            ed = r["effects_dead_km"]
            ee = r["effects_energy_wh"]

            def _g(scn, key):
                return f"{scn[key]:.3f}" if scn else ""

            writer.writerow({
                "category": r["category"],
                "field": r["field"],
                "is_dynamic": int(r["is_dynamic_field"]),
                "heuristic_angle": f"{r['heuristic_angle_deg']:.1f}",
                "grid_angle": f"{r['grid_angle_deg']:.0f}",
                "S1_dead_km": _g(s["S1_heur_static"], "dead_km"),
                "S2_dead_km": _g(s["S2_heur_dynamic"], "dead_km"),
                "S3_dead_km": _g(s["S3_opt_static"], "dead_km"),
                "S4_dead_km": _g(s["S4_opt_dynamic"], "dead_km"),
                "S1_energy_wh": _g(s["S1_heur_static"], "energy_nonprod_wh"),
                "S2_energy_wh": _g(s["S2_heur_dynamic"], "energy_nonprod_wh"),
                "S3_energy_wh": _g(s["S3_opt_static"], "energy_nonprod_wh"),
                "S4_energy_wh": _g(s["S4_opt_dynamic"], "energy_nonprod_wh"),
                "dead_opt_static_pct": f"{ed['optimization_static_pct']:.2f}",
                "dead_dyn_heur_pct": f"{ed.get('dynamic_heuristic_pct', 0):.2f}" if r["is_dynamic_field"] else "",
                "dead_opt_dyn_pct": f"{ed.get('optimization_dynamic_pct', 0):.2f}" if r["is_dynamic_field"] else "",
                "dead_combined_pct": f"{ed.get('combined_pct', 0):.2f}" if r["is_dynamic_field"] else "",
                "dead_interaction_pct": f"{ed.get('interaction_pct', 0):.2f}" if r["is_dynamic_field"] else "",
                "energy_opt_static_pct": f"{ee['optimization_static_pct']:.2f}",
                "energy_dyn_heur_pct": f"{ee.get('dynamic_heuristic_pct', 0):.2f}" if r["is_dynamic_field"] else "",
                "energy_opt_dyn_pct": f"{ee.get('optimization_dynamic_pct', 0):.2f}" if r["is_dynamic_field"] else "",
                "energy_combined_pct": f"{ee.get('combined_pct', 0):.2f}" if r["is_dynamic_field"] else "",
                "energy_interaction_pct": f"{ee.get('interaction_pct', 0):.2f}" if r["is_dynamic_field"] else "",
            })


if __name__ == "__main__":
    main()
