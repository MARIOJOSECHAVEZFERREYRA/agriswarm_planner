"""Tick-by-tick mission simulation over pre-built UAV and UGV routes."""

import asyncio
import math
import time
from collections.abc import AsyncGenerator

from backend.algorithms.energy.energy_model import DroneEnergyModel
from backend.schemas.simulation import SegmentType, SimulationFrame, VehicleSimState
from backend.services.route_builder_service import VehicleRoute, RouteSegment


def _cumulative_energy_frac(seg: RouteSegment, t_norm: float, drone) -> float:
    """Return consumed energy fraction at normalized time ``t_norm``."""
    if (seg.t_acc_s <= 0.0 and seg.t_dec_s <= 0.0) or seg.duration_s <= 0.0:
        return t_norm

    t = t_norm * seg.duration_s
    t_a = seg.t_acc_s
    t_d = seg.t_dec_s
    t_c = max(0.0, seg.duration_s - t_a - t_d)

    k_a = getattr(drone, "power_accel_factor", 1.15) if drone else 1.15
    k_d = getattr(drone, "power_decel_factor", 1.05) if drone else 1.05
    total_eff = k_a * t_a + t_c + k_d * t_d
    if total_eff <= 0.0:
        return t_norm

    if t <= t_a:
        eff = k_a * t
    elif t <= t_a + t_c:
        eff = k_a * t_a + (t - t_a)
    else:
        eff = k_a * t_a + t_c + k_d * (t - t_a - t_c)

    return eff / total_eff


class VehicleCursor:
    """Track the current segment, progress, and resources of one vehicle."""

    def __init__(
        self,
        route: VehicleRoute,
        initial_energy_wh: float,
        initial_reagent_l: float,
        drone=None,
    ):
        self.route = route
        self.segment_index = 0
        self.segment_progress = 0.0
        self.energy_wh = initial_energy_wh
        self.reagent_l = initial_reagent_l
        self.initial_energy_wh = initial_energy_wh
        self.initial_reagent_l = initial_reagent_l
        self._drone = drone
        self.done = False
        self.sim_time_s = 0.0

    @property
    def current_segment(self) -> RouteSegment | None:
        if self.segment_index >= len(self.route.segments):
            return None
        return self.route.segments[self.segment_index]

    def advance(self, dt_real: float, playback_speed: float) -> None:
        """Advance the cursor by ``dt_real * playback_speed`` simulated seconds."""
        if self.done:
            return

        dt_sim = dt_real * playback_speed
        self.sim_time_s += dt_sim
        time_remaining = dt_sim

        while time_remaining > 1e-6 and self.segment_index < len(self.route.segments):
            seg = self.route.segments[self.segment_index]

            seg_duration = max(0.001, seg.duration_s)
            time_in_seg = self.segment_progress * seg_duration
            time_left_in_seg = seg_duration - time_in_seg

            if time_left_in_seg <= time_remaining + 1e-6:
                t_norm_before = self.segment_progress
                self.segment_progress = 1.0

                e_frac_before = _cumulative_energy_frac(seg, t_norm_before, self._drone)
                self.energy_wh -= seg.energy_cost_wh * (1.0 - e_frac_before)

                fraction_remaining = max(0.0, 1.0 - (time_in_seg / seg_duration))
                self.reagent_l -= seg.reagent_consumed_l * fraction_remaining

                time_remaining -= time_left_in_seg
                prev_type = seg.segment_type

                self.segment_index += 1
                self.segment_progress = 0.0

                if prev_type == SegmentType.service and self.segment_index < len(self.route.segments):
                    self.energy_wh = self.initial_energy_wh
                    self.reagent_l = self.initial_reagent_l
            else:
                advance_fraction = time_remaining / seg_duration
                t_norm_before = self.segment_progress
                self.segment_progress = min(1.0, self.segment_progress + advance_fraction)

                e_frac_before = _cumulative_energy_frac(seg, t_norm_before, self._drone)
                e_frac_after = _cumulative_energy_frac(seg, self.segment_progress, self._drone)
                self.energy_wh -= seg.energy_cost_wh * (e_frac_after - e_frac_before)

                self.reagent_l -= seg.reagent_consumed_l * advance_fraction

                time_remaining = 0.0

        if self.segment_index >= len(self.route.segments):
            self.done = True

    def get_state(self, vehicle_id: str, drone) -> VehicleSimState:
        """Interpolate the current route state into an API frame."""
        if self.done or self.segment_index >= len(self.route.segments):
            last_seg = self.route.segments[-1]
            pos = last_seg.p2
            return VehicleSimState(
                vehicle_id=vehicle_id,
                x=pos[0],
                y=pos[1],
                heading=0.0,
                speed=0.0,
                segment_type=SegmentType.service,
                cycle_index=len(self.route.segments) - 1,
                waypoint_index=len(self.route.segments) - 1,
                battery_pct=0.0,
                energy_remaining_wh=max(0.0, self.energy_wh),
                reagent_l=max(0.0, self.reagent_l),
                pump_active=False,
                is_done=True,
            )

        seg = self.current_segment
        if seg is None:
            self.done = True
            return VehicleSimState(
                vehicle_id=vehicle_id,
                x=0.0,
                y=0.0,
                heading=0.0,
                speed=0.0,
                segment_type=SegmentType.service,
                cycle_index=0,
                waypoint_index=0,
                battery_pct=0.0,
                energy_remaining_wh=0.0,
                reagent_l=0.0,
                pump_active=False,
                is_done=True,
            )

        progress = max(0.0, min(1.0, self.segment_progress))

        if seg.distance_m > 1e-6:
            x = seg.p1[0] + (seg.p2[0] - seg.p1[0]) * progress
            y = seg.p1[1] + (seg.p2[1] - seg.p1[1]) * progress
        else:
            x, y = seg.p1

        if seg.distance_m > 1e-6:
            heading = math.degrees(math.atan2(seg.p2[1] - seg.p1[1], seg.p2[0] - seg.p1[0])) % 360
        else:
            heading = 0.0

        if seg.duration_s > 0:
            speed = seg.distance_m / seg.duration_s
        else:
            speed = 0.0

        if drone:
            battery_capacity = drone.battery_capacity_wh
            reserve_pct = drone.battery_reserve_pct
            usable_capacity = battery_capacity * (1.0 - reserve_pct / 100.0)
            battery_pct = (self.energy_wh / usable_capacity * 100.0) if usable_capacity > 0 else 0.0
            battery_pct = max(0.0, min(100.0, battery_pct))
        else:
            battery_pct = 100.0

        pump_active = seg.segment_type == SegmentType.spray
        energy_to_report = self.energy_wh
        reagent_to_report = self.reagent_l

        if seg.segment_type == SegmentType.service:
            energy_to_report = (
                drone.battery_capacity_wh * (1.0 - drone.battery_reserve_pct / 100.0)
                if drone
                else self.energy_wh
            )
            reagent_to_report = drone.mass_tank_full_kg if drone else self.reagent_l
            battery_pct = 100.0

        return VehicleSimState(
            vehicle_id=vehicle_id,
            x=x,
                y=y,
                heading=heading,
                speed=speed,
                segment_type=seg.segment_type,
                cycle_index=seg.cycle_index,
                waypoint_index=self.segment_index,
                battery_pct=battery_pct,
                energy_remaining_wh=max(0.0, energy_to_report),
                reagent_l=max(0.0, reagent_to_report),
            pump_active=pump_active,
            is_done=False,
        )


class SimulationState:
    """Mutable simulation controls shared with the websocket task."""

    def __init__(self):
        self.playback_speed = 1.0


async def stream_simulation(
    uav_route: VehicleRoute,
    ugv_route: VehicleRoute,
    drone,
    state: SimulationState,
    interval_ms: int = 200,
) -> AsyncGenerator[SimulationFrame, None]:
    """Yield simulation frames until both vehicles finish their routes."""
    usable_wh = DroneEnergyModel(drone).usable_energy_wh()
    uav_cursor = VehicleCursor(
        uav_route,
        initial_energy_wh=usable_wh,
        initial_reagent_l=drone.mass_tank_full_kg,
        drone=drone,
    )
    ugv_cursor = VehicleCursor(
        ugv_route,
        initial_energy_wh=float("inf"),
        initial_reagent_l=0.0,
    )

    dt = interval_ms / 1000.0

    while not (uav_cursor.done and ugv_cursor.done):
        t_ms = int(time.time() * 1000)
        sim_time = uav_cursor.sim_time_s
        current_speed = state.playback_speed

        uav_cursor.advance(dt, current_speed)
        ugv_cursor.advance(dt, current_speed)

        frame = SimulationFrame(
            timestamp_ms=t_ms,
            sim_time_s=sim_time,
            vehicles=[
                uav_cursor.get_state("uav", drone),
                ugv_cursor.get_state("ugv", drone),
            ],
            playback_speed=current_speed,
        )
        yield frame
        await asyncio.sleep(dt)
