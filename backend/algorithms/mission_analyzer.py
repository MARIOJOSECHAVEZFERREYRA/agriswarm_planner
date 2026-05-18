import numpy as np
from .energy.energy_model import DroneEnergyModel


class MissionAnalyzer:
    """
    Analyzes missions (Static and Dynamic) and generates logistic plans.

    All methods accept a Drone ORM instance for parameters.
    """

    @staticmethod
    def calculate_comprehensive_metrics(cycles, polygon, drone, flow_l_min=None):
        """
        Calculates detailed metrics for the mission report.

        Uses drone parameters for service time and the three-phase
        kinematic profile (via DroneEnergyModel) for accurate flight
        time estimation.

        flow_l_min: effective flow rate in L/min used during the mission.
                    Defaults to drone.spray_flow_rate_lpm if not provided.
        """
        energy_model = DroneEnergyModel(drone)

        total_dist = 0.0
        spray_dist = 0.0
        ferry_dist = 0.0
        deadhead_dist = 0.0
        flight_time_sec = 0.0
        spray_area_m2 = 0.0
        energy_total_wh = 0.0
        energy_spray_wh = 0.0
        energy_deadhead_wh = 0.0
        energy_ferry_wh = 0.0
        tank_full_l = float(drone.mass_tank_full_kg)

        for c in cycles:
            swath = float(c.get('swath_width', 0.0))
            for s in c['segments']:
                d = float(np.linalg.norm(
                    np.array(s['p1'][:2]) - np.array(s['p2'][:2])
                ))
                total_dist += d
                seg_type = s.get('segment_type', 'ferry')
                if seg_type == 'sweep':
                    spray_dist += d
                    spray_area_m2 += d * swath
                    flight_time_sec += energy_model.time_straight(d)
                    e = energy_model.energy_straight(d, tank_full_l)
                    energy_spray_wh += e
                    energy_total_wh += e
                elif seg_type == 'deadhead':
                    deadhead_dist += d
                    flight_time_sec += energy_model.time_transit(d)
                    e = energy_model.energy_transit(d, tank_full_l)
                    energy_deadhead_wh += e
                    energy_total_wh += e
                else:
                    ferry_dist += d
                    flight_time_sec += energy_model.time_transit(d)
                    e = energy_model.energy_transit(d, tank_full_l)
                    energy_ferry_wh += e
                    energy_total_wh += e

        area_m2 = polygon.area
        area_ha = area_m2 / 10000.0

        flight_time_min = flight_time_sec / 60.0
        n_cycles = len(cycles)

        # Service time: use drone.service_time_s per inter-cycle stop.
        # First cycle departs with full load; last cycle has no reload
        # after it, so there are (n_cycles - 1) service stops.
        service_time_s = float(drone.service_time_s)
        n_service_stops = max(n_cycles - 1, 0)
        service_time_min = (n_service_stops * service_time_s) / 60.0

        # In dynamic mode cycles may carry rv_wait_s (UAV idle time
        # waiting for UGV to arrive at the rendezvous point).
        rv_wait_total_s = sum(
            float(c.get('rv_wait_s', 0.0)) for c in cycles
        )
        rv_wait_min = rv_wait_total_s / 60.0

        total_op_time_min = flight_time_min + service_time_min + rv_wait_min

        prod_ha_hr = 0.0
        if total_op_time_min > 0:
            prod_ha_hr = area_ha / (total_op_time_min / 60.0)

        effective_flow = (
            flow_l_min if flow_l_min is not None
            else drone.spray_flow_rate_lpm
        )
        spray_time_min = (spray_dist / drone.speed_cruise_ms) / 60.0
        total_vol_l = spray_time_min * effective_flow

        real_dosage = 0.0
        if area_ha > 0:
            real_dosage = total_vol_l / area_ha

        non_spray_dist = ferry_dist + deadhead_dist

        return {
            "area_ha": area_ha,
            "flight_time_min": flight_time_min,
            "service_time_min": service_time_min,
            "total_op_time_min": total_op_time_min,
            "productivity_ha_hr": prod_ha_hr,
            "real_dosage_l_ha": real_dosage,
            "spray_dist_km": spray_dist / 1000.0,
            "ferry_dist_km": ferry_dist / 1000.0,
            "dead_dist_km": non_spray_dist / 1000.0,
            "deadhead_dist_km": deadhead_dist / 1000.0,
            "efficiency_ratio": (
                (spray_area_m2 / area_m2 * 100) if area_m2 > 0 else 0
            ),
            "energy_total_wh": energy_total_wh,
            "energy_spray_wh": energy_spray_wh,
            "energy_deadhead_wh": energy_deadhead_wh,
            "energy_ferry_wh": energy_ferry_wh,
        }

    @staticmethod
    def plan_logistics(cycles, drone, flow_l_min=None):
        """
        Generates the resource plan (battery packs, liquid mix, stop list).

        flow_l_min: effective flow rate in L/min used during the mission.
                    Defaults to drone.spray_flow_rate_lpm if not provided.
        """
        tank_l = drone.mass_tank_full_kg

        energy_model = DroneEnergyModel(drone)
        avg_power_w = energy_model.spray_power(tank_l)
        flight_time_min = energy_model.usable_energy_wh() / avg_power_w * 60.0

        charge_time_min = drone.battery_charge_time_min
        packs_needed = 1 + int(np.ceil(charge_time_min / flight_time_min))

        effective_flow = (
            flow_l_min if flow_l_min is not None
            else drone.spray_flow_rate_lpm
        )
        work_speed_ms = drone.speed_cruise_ms

        stops = []
        total_liter_mix = 0.0

        for i, cycle in enumerate(cycles):
            stop_type = "Start" if i == 0 else "Stop {}".format(i)

            spray_dist_m = 0.0
            for s in cycle['segments']:
                if s['spraying']:
                    d = np.linalg.norm(
                        np.array(s['p1'][:2]) - np.array(s['p2'][:2])
                    )
                    spray_dist_m += d

            spray_time_min = (spray_dist_m / work_speed_ms) / 60.0
            liters_consumed = min(spray_time_min * effective_flow, tank_l)
            total_liter_mix += liters_consumed

            if i == 0:
                action_desc = "Fill Tank ({:.1f}L)".format(liters_consumed)
            else:
                action_desc = "Bat. Swap + Refill {:.1f}L".format(
                    liters_consumed
                )

            stops.append({
                "name": stop_type,
                "action": action_desc,
                "notes": "Coverage: {:.0f}m".format(spray_dist_m),
            })

        return {
            "battery_packs": packs_needed,
            "total_mix_l": total_liter_mix,
            "stops": stops,
        }
