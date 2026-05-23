"""Log every find_best_rendezvous call during a real plan_dynamic_cycles
run, with full cost breakdown.

Gate ensures we only capture calls made during plan_dynamic_cycles (not
during the grid search simulator's 180 per-angle evaluations).
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
    boundary = [tuple(p) for p in data["boundary"]]
    if boundary[0] == boundary[-1]:
        boundary = boundary[:-1]
    return {
        "boundary": boundary,
        "base_point": tuple(data["base_point"]),
        "ugv_polyline": [tuple(p) for p in data["ugv_polyline"]],
        "ugv_speed": float(data.get("ugv_speed", 2.0)),
        "ugv_t_service": float(data.get("ugv_t_service", 300.0)),
    }


LOG = {"enabled": False, "calls": []}
ORIG_FIND = RendezvousPlanner.find_best_rendezvous


def _patched_find(self, uav_pos, uav_energy_rem, ugv_distance_along,
                  t_current, v_uav, transit_energy_fn, e_reserve):
    result = ORIG_FIND(self, uav_pos, uav_energy_rem,
                       ugv_distance_along, t_current, v_uav,
                       transit_energy_fn, e_reserve)
    if not LOG["enabled"]:
        return result

    v = max(float(v_uav), 1e-9)
    cost_table = []
    for i, cand in enumerate(self._candidates):
        cx, cy = cand["point"]
        da = cand["distance_along"]
        d_uav = math.hypot(cx - uav_pos[0], cy - uav_pos[1])
        e_need = transit_energy_fn(d_uav)
        feas_e = e_need <= uav_energy_rem - e_reserve
        feas_m = da >= ugv_distance_along - 1e-3
        ft = d_uav / v
        d_ugv = max(0.0, da - ugv_distance_along)
        tugv = d_ugv / self.v_ugv
        t_u = t_current + ft
        t_g = t_current + tugv
        tw = max(0.0, t_g - t_u)
        cost = self.alpha * tw + self.beta * ft + self.gamma * tugv
        cost_table.append({
            "idx": i, "d_along": da, "d_uav": d_uav, "d_ugv": d_ugv,
            "t_wait": tw, "cost": cost, "feas_e": feas_e, "feas_m": feas_m,
        })
    LOG["calls"].append({
        "uav_pos": uav_pos,
        "ugv_da": ugv_distance_along,
        "e_rem": uav_energy_rem,
        "e_reserve": e_reserve,
        "gamma": self.gamma,
        "chosen": result.get("point") if result.get("feasible") else None,
        "chosen_da": result.get("distance_along") if result.get("feasible") else None,
        "cost_table": cost_table,
    })
    return result


def _run_grid(field):
    planner = DynamicMissionPlanner(
        ugv_polyline=field["ugv_polyline"],
        ugv_speed=field["ugv_speed"],
        ugv_t_service=field["ugv_t_service"],
    )
    engine = create_engine("sqlite:///agriswarm.db")
    db = sessionmaker(bind=engine)()
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
    return drone, result


def _plan_with_gamma(field, drone, result, gamma):
    rp = RendezvousPlanner(
        ugv_polyline=field["ugv_polyline"],
        v_ugv=field["ugv_speed"],
        t_service=field["ugv_t_service"],
        gamma=gamma,
    )
    segmenter = MissionSegmenter(
        drone, target_rate_l_ha=20.0, work_speed_kmh=5.0 * 3.6,
        swath_width=5.0, energy_model=DroneEnergyModel(drone),
    )
    LOG["calls"] = []
    LOG["enabled"] = True
    try:
        plan = rp.plan_dynamic_cycles(
            segmenter, result.get("safe_polygon"),
            result.get("route_segments"),
        )
    finally:
        LOG["enabled"] = False
    return plan, rp


def _find_cut_decisions(calls, cycles):
    """Identify which calls correspond to real cut decisions.

    A cut call is one whose chosen (distance_along) matches a cycle's
    base_point distance along the polyline. Return list of
    (cycle_idx, call_idx, call).
    """
    # Build index of cycles with rv_wait_s (they have a real RV).
    rv_cycles = [(i, c) for i, c in enumerate(cycles) if "rv_wait_s" in c]
    decisions = []
    for cyc_idx, cycle in rv_cycles:
        bp = cycle["base_point"]
        for j, call in enumerate(calls):
            ch = call["chosen"]
            if ch is None:
                continue
            if math.hypot(bp[0] - ch[0], bp[1] - ch[1]) < 1e-6:
                # The rv_after probes also produce a chosen point; we
                # want the LAST call whose uav_pos equals the cut point
                # (uav_pos before the DH to RV).
                decisions.append((cyc_idx, j, call))
                break
    return decisions


def main():
    RendezvousPlanner.find_best_rendezvous = _patched_find

    field = _load_field(FIELD)
    print("Phase 1: grid search to get route...")
    drone, result = _run_grid(field)
    print(f"Winner angle: {result.get('best_angle')}  "
          f"route: {len(result.get('route_segments', []))} segments")
    print()

    for gamma in [0.3, 100.0]:
        print(f"=== gamma = {gamma} ===")
        plan, rp = _plan_with_gamma(field, drone, result, gamma)
        cycles = plan.get("cycles", [])
        print(f"cycles produced: {len(cycles)}")
        rv_cycles = [c for c in cycles if "rv_wait_s" in c]
        print(f"cycles with RV:  {len(rv_cycles)}")
        print(f"fbr calls logged: {len(LOG['calls'])}")

        decisions = _find_cut_decisions(LOG["calls"], cycles)
        print(f"decisions identified: {len(decisions)}")
        print()
        # Print the FIRST cut's decision table.
        if decisions:
            cyc_idx, call_idx, call = decisions[0]
            print(f"-- cut #0 (cycle[{cyc_idx}], fbr_call[{call_idx}]) --")
            print(f"UAV pos:       ({call['uav_pos'][0]:.1f}, {call['uav_pos'][1]:.1f})")
            print(f"UGV d_along:   {call['ugv_da']:.1f}")
            print(f"UAV energy:    {call['e_rem']:.0f} Wh (reserve {call['e_reserve']:.0f})")
            print(f"Chosen d_along: {call['chosen_da']:.1f}")
            print()
            print(f"  {'idx':>3} {'d_along':>8} {'d_uav':>7} {'d_ugv':>7} "
                  f"{'t_wait':>7} {'cost':>10} {'mono':>4} {'eOK':>3}")
            for e in call["cost_table"]:
                m = "Y" if e["feas_m"] else "N"
                en = "Y" if e["feas_e"] else "N"
                print(f"  {e['idx']:>3} {e['d_along']:>8.1f} {e['d_uav']:>7.1f} "
                      f"{e['d_ugv']:>7.1f} {e['t_wait']:>7.1f} {e['cost']:>10.1f} "
                      f"{m:>4} {en:>3}")
            # Also print the chosen candidate summary for cuts 1..5 to
            # see if gamma flipped anything downstream.
            print()
            print("Summary of first 6 cuts:")
            print(f"  {'cut':>3}  {'uav_pos':>18}  {'ugv_da':>7}  {'chosen_da':>9}")
            for i, (_, _, c) in enumerate(decisions[:6]):
                ux, uy = c["uav_pos"]
                cda = c["chosen_da"] if c["chosen_da"] is not None else float("nan")
                print(f"  {i:>3}  ({ux:>7.1f},{uy:>7.1f})  {c['ugv_da']:>7.1f}  {cda:>9.1f}")
        print()


if __name__ == "__main__":
    main()
