"""Build simulated UAV and UGV routes from planned mission cycles."""

import math
from dataclasses import dataclass
from typing import Optional

from shapely.geometry import Point, Polygon

from backend.algorithms.energy.energy_model import DroneEnergyModel
from backend.schemas.simulation import SegmentType


@dataclass
class RouteSegment:
    """A single segment in a vehicle's route."""

    p1: tuple[float, float]
    p2: tuple[float, float]
    segment_type: SegmentType
    cycle_index: int
    distance_m: float
    duration_s: float
    energy_cost_wh: float
    reagent_consumed_l: float
    t_acc_s: float = 0.0
    t_dec_s: float = 0.0


@dataclass
class VehicleRoute:
    """Complete route for a vehicle."""

    vehicle_id: str
    segments: list[RouteSegment]
    total_duration_s: float
    total_energy_wh: float
    total_reagent_l: float


class UAVRouteBuilder:
    """Build the simulated UAV route for each mission cycle."""

    def __init__(self, drone, work_polygon: Polygon):
        self.drone = drone
        self.energy_model = DroneEnergyModel(drone)
        self.polygon = work_polygon

    def build(
        self,
        mission_cycles: list[dict],
        service_duration_s: Optional[float] = None,
    ) -> VehicleRoute:
        """Build a route that visits every cycle segment plus service stops."""
        service_duration_s = (
            float(service_duration_s)
            if service_duration_s is not None
            else float(self.drone.service_time_s)
        )

        segments = []
        total_energy = 0.0
        total_reagent = 0.0
        total_duration = 0.0

        for cycle_idx, cycle in enumerate(mission_cycles):
            base_point = tuple(cycle["base_point"])
            cycle_segments = cycle.get("segments", [])

            # Current energy/reagent state at start of cycle
            energy_remaining = self.energy_model.usable_energy_wh()
            reagent_remaining = self.drone.mass_tank_full_kg

            for seg in cycle_segments:
                p1 = tuple(seg["p1"][:2])
                p2 = tuple(seg["p2"][:2])
                is_spraying = seg["spraying"]
                explicit = seg.get("segment_type", "")

                if is_spraying:
                    seg_type = SegmentType.spray
                elif explicit == "deadhead":
                    seg_type = SegmentType.deadhead
                elif explicit == "ferry":
                    seg_type = SegmentType.ferry
                else:
                    seg_type = self._classify_segment(p1, p2, is_spraying, base_point)
                distance = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

                if seg_type == SegmentType.spray:
                    duration = self.energy_model.time_straight(distance)
                    energy_cost = self.energy_model.energy_straight(distance, reagent_remaining)
                    reagent_cost = self.energy_model.reagent_consumed(distance)
                    t_a, _, t_d, _ = self.energy_model._straight_profile(
                        distance,
                        self.drone.speed_cruise_ms,
                        self.drone.accel_horizontal_ms2,
                        self.drone.decel_horizontal_ms2,
                    )
                else:
                    duration = self.energy_model.time_transit(distance)
                    energy_cost = self.energy_model.energy_transit(distance, reagent_remaining)
                    reagent_cost = 0.0
                    t_a, _, t_d, _ = self.energy_model._straight_profile(
                        distance,
                        self.drone.speed_max_ms,
                        self.drone.accel_horizontal_ms2,
                        self.drone.decel_horizontal_ms2,
                    )

                route_seg = RouteSegment(
                    p1=p1,
                    p2=p2,
                    segment_type=seg_type,
                    cycle_index=cycle_idx,
                    distance_m=distance,
                    duration_s=duration,
                    energy_cost_wh=energy_cost,
                    reagent_consumed_l=reagent_cost,
                    t_acc_s=t_a,
                    t_dec_s=t_d,
                )
                segments.append(route_seg)

                total_energy += energy_cost
                total_reagent += reagent_cost
                total_duration += duration

                energy_remaining -= energy_cost
                reagent_remaining -= reagent_cost

            service_seg = RouteSegment(
                p1=base_point,
                p2=base_point,
                segment_type=SegmentType.service,
                cycle_index=cycle_idx,
                distance_m=0.0,
                duration_s=service_duration_s,
                energy_cost_wh=0.0,
                reagent_consumed_l=0.0,
            )
            segments.append(service_seg)
            total_duration += service_duration_s

        return VehicleRoute(
            vehicle_id="uav",
            segments=segments,
            total_duration_s=total_duration,
            total_energy_wh=total_energy,
            total_reagent_l=total_reagent,
        )

    def _classify_segment(
        self,
        p1: tuple[float, float],
        p2: tuple[float, float],
        is_spraying: bool,
        base_point: tuple[float, float],
    ) -> SegmentType:
        """Classify a non-spray leg as ferry or deadhead."""
        if is_spraying:
            return SegmentType.spray

        base_tolerance_m = 0.5
        p1_dist_to_base = math.hypot(p1[0] - base_point[0], p1[1] - base_point[1])
        p2_dist_to_base = math.hypot(p2[0] - base_point[0], p2[1] - base_point[1])

        if p1_dist_to_base < base_tolerance_m or p2_dist_to_base < base_tolerance_m:
            return SegmentType.deadhead

        mid_x = (p1[0] + p2[0]) / 2
        mid_y = (p1[1] + p2[1]) / 2
        mid_pt = Point(mid_x, mid_y)

        if self.polygon.contains(mid_pt) or self.polygon.boundary.distance(mid_pt) < 0.1:
            return SegmentType.ferry

        return SegmentType.deadhead


class UGVRouteBuilder:
    """Build static or mobile UGV routes aligned to the UAV timeline."""

    def build_static(
        self,
        mission_cycles: list[dict],
        uav_total_duration_s: float,
        ugv_garage: Optional[tuple[float, float]] = None,
    ) -> VehicleRoute:
        """Build a waiting route for a fixed-base UGV."""
        if not mission_cycles:
            return VehicleRoute(
                vehicle_id="ugv",
                segments=[],
                total_duration_s=0.0,
                total_energy_wh=0.0,
                total_reagent_l=0.0,
            )

        base_point = tuple(mission_cycles[0]["base_point"])

        segments = [
            RouteSegment(
                p1=base_point,
                p2=base_point,
                segment_type=SegmentType.service,
                cycle_index=0,
                distance_m=0.0,
                duration_s=uav_total_duration_s,
                energy_cost_wh=0.0,
                reagent_consumed_l=0.0,
            )
        ]

        return VehicleRoute(
            vehicle_id="ugv",
            segments=segments,
            total_duration_s=uav_total_duration_s,
            total_energy_wh=0.0,
            total_reagent_l=0.0,
        )

    def build_mobile(
        self,
        mission_cycles: list[dict],
        uav_route: "VehicleRoute",
        ugv_polyline: list,
        ugv_speed: float,
        ugv_t_service: float,
    ) -> VehicleRoute:
        """Build a moving UGV route that follows the mission rendezvous plan."""
        if not mission_cycles or not ugv_polyline or len(ugv_polyline) < 2:
            return self.build_static(mission_cycles, uav_route.total_duration_s)

        ugv_speed = max(float(ugv_speed), 0.1)
        cycle_count = len(mission_cycles)
        ugv_path = [(float(point[0]), float(point[1])) for point in ugv_polyline]

        rendezvous_count = max(0, cycle_count - 1)
        rendezvous_points: list[tuple[float, float]] = [
            (float(mission_cycles[i]["base_point"][0]),
             float(mission_cycles[i]["base_point"][1]))
            for i in range(rendezvous_count)
        ]

        rendezvous_distances = [
            self._find_distance_along(ugv_path, point)
            for point in rendezvous_points
        ]
        total_polyline_length = self._polyline_length(ugv_path)

        elapsed_s = 0.0
        service_windows: list[tuple[float, float]] = []
        for seg in uav_route.segments:
            if seg.segment_type == SegmentType.service:
                service_windows.append((elapsed_s, seg.duration_s))
            elapsed_s += seg.duration_s

        rendezvous_windows = service_windows[:rendezvous_count]

        total_uav_dur = uav_route.total_duration_s
        segments: list[RouteSegment] = []

        if rendezvous_count == 0 or not rendezvous_windows:
            self._append_polyline_transit(segments, ugv_path, total_uav_dur, ugv_speed, cycle_index=0)
        else:
            first_service_start_s, _ = rendezvous_windows[0]
            initial_path = self._subpath_along(ugv_path, 0.0, rendezvous_distances[0])
            self._append_polyline_transit(
                segments,
                initial_path,
                first_service_start_s,
                ugv_speed,
                cycle_index=0,
            )

            for i in range(rendezvous_count):
                service_start_s, service_duration_s = rendezvous_windows[i]

                segments.append(RouteSegment(
                    p1=rendezvous_points[i], p2=rendezvous_points[i],
                    segment_type=SegmentType.service,
                    cycle_index=i,
                    distance_m=0.0,
                    duration_s=max(service_duration_s, 0.001),
                    energy_cost_wh=0.0,
                    reagent_consumed_l=0.0,
                ))

                if i + 1 < rendezvous_count:
                    next_service_start_s, _ = rendezvous_windows[i + 1]
                    available_s = max(
                        next_service_start_s - (service_start_s + service_duration_s),
                        0.001,
                    )
                    next_path = self._subpath_along(
                        ugv_path,
                        rendezvous_distances[i],
                        rendezvous_distances[i + 1],
                    )
                    self._append_polyline_transit(
                        segments,
                        next_path,
                        available_s,
                        ugv_speed,
                        cycle_index=i,
                    )

            last_service_start_s, last_service_duration_s = rendezvous_windows[-1]
            remaining_s = total_uav_dur - (last_service_start_s + last_service_duration_s)
            if remaining_s > 0.001:
                final_path = self._subpath_along(
                    ugv_path,
                    rendezvous_distances[-1],
                    total_polyline_length,
                )
                self._append_polyline_transit(
                    segments,
                    final_path,
                    remaining_s,
                    ugv_speed,
                    cycle_index=rendezvous_count - 1,
                )

        total_duration = sum(seg.duration_s for seg in segments)
        return VehicleRoute(
            vehicle_id="ugv",
            segments=segments,
            total_duration_s=total_duration,
            total_energy_wh=0.0,
            total_reagent_l=0.0,
        )

    @staticmethod
    def _polyline_length(polyline: list) -> float:
        """Total Euclidean length of an ordered list of (x, y) points."""
        total = 0.0
        for i in range(len(polyline) - 1):
            p1, p2 = polyline[i], polyline[i + 1]
            total += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        return total

    @staticmethod
    def _interp_at_distance(polyline: list, d: float) -> tuple:
        """Interpolate (x, y) at accumulated distance d along polyline."""
        d = max(0.0, d)
        accumulated = 0.0
        for i in range(len(polyline) - 1):
            p1 = polyline[i]
            p2 = polyline[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if accumulated + seg_len >= d - 1e-9:
                if seg_len < 1e-9:
                    return (float(p1[0]), float(p1[1]))
                t = max(0.0, min(1.0, (d - accumulated) / seg_len))
                return (p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]))
            accumulated += seg_len
        return (float(polyline[-1][0]), float(polyline[-1][1]))

    @staticmethod
    def _find_distance_along(polyline: list, point: tuple) -> float:
        """Project a point onto a polyline and return accumulated distance."""
        min_dist = float("inf")
        best_d = 0.0
        accumulated = 0.0
        for i in range(len(polyline) - 1):
            p1 = polyline[i]
            p2 = polyline[i + 1]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-18:
                continue
            seg_len = math.sqrt(seg_len_sq)
            t = max(
                0.0,
                min(1.0, ((point[0] - p1[0]) * dx + (point[1] - p1[1]) * dy) / seg_len_sq),
            )
            px = p1[0] + t * dx
            py = p1[1] + t * dy
            dist = math.hypot(point[0] - px, point[1] - py)
            if dist < min_dist:
                min_dist = dist
                best_d = accumulated + t * seg_len
            accumulated += seg_len
        return best_d

    @staticmethod
    def _subpath_along(polyline: list, d_from: float, d_to: float) -> list:
        """Extract the polyline subpath between two accumulated distances."""
        if d_to <= d_from + 1e-6:
            pt = UGVRouteBuilder._interp_at_distance(polyline, d_from)
            return [pt, pt]

        p_start = UGVRouteBuilder._interp_at_distance(polyline, d_from)
        p_end = UGVRouteBuilder._interp_at_distance(polyline, d_to)
        path = [p_start]
        accumulated = 0.0

        for i in range(len(polyline) - 1):
            p1 = polyline[i]
            p2 = polyline[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            vertex_d = accumulated + seg_len
            if d_from + 1e-6 < vertex_d < d_to - 1e-6:
                path.append((float(p2[0]), float(p2[1])))
            accumulated = vertex_d

        if math.hypot(path[-1][0] - p_end[0], path[-1][1] - p_end[1]) > 1e-6:
            path.append(p_end)
        return path

    @staticmethod
    def _append_polyline_transit(
        segments: list,
        path: list,
        available_s: float,
        ugv_speed: float,
        cycle_index: int,
    ) -> None:
        """Append UGV motion or wait segments to fill an available time window."""
        available_s = max(float(available_s), 0.001)

        if len(path) < 2:
            pt = tuple(path[0]) if path else (0.0, 0.0)
            segments.append(RouteSegment(
                p1=pt, p2=pt,
                segment_type=SegmentType.service,
                cycle_index=cycle_index,
                distance_m=0.0, duration_s=available_s,
                energy_cost_wh=0.0, reagent_consumed_l=0.0,
            ))
            return

        sub_dists = [
            math.hypot(float(path[i + 1][0]) - float(path[i][0]),
                       float(path[i + 1][1]) - float(path[i][1]))
            for i in range(len(path) - 1)
        ]
        total_d = sum(sub_dists)

        if total_d < 0.1:
            segments.append(RouteSegment(
                p1=tuple(path[0]), p2=tuple(path[-1]),
                segment_type=SegmentType.service,
                cycle_index=cycle_index,
                distance_m=0.0, duration_s=available_s,
                energy_cost_wh=0.0, reagent_consumed_l=0.0,
            ))
            return

        actual_travel_s = total_d / ugv_speed

        if actual_travel_s <= available_s:
            for i in range(len(path) - 1):
                d = sub_dists[i]
                if d < 1e-9:
                    continue
                segments.append(RouteSegment(
                    p1=tuple(path[i]), p2=tuple(path[i + 1]),
                    segment_type=SegmentType.deadhead,
                    cycle_index=cycle_index,
                    distance_m=d,
                    duration_s=d / ugv_speed,
                    energy_cost_wh=0.0, reagent_consumed_l=0.0,
                ))
            wait_dur = available_s - actual_travel_s
            if wait_dur > 0.001:
                segments.append(RouteSegment(
                    p1=tuple(path[-1]), p2=tuple(path[-1]),
                    segment_type=SegmentType.service,
                    cycle_index=cycle_index,
                    distance_m=0.0, duration_s=wait_dur,
                    energy_cost_wh=0.0, reagent_consumed_l=0.0,
                ))
        else:
            for i in range(len(path) - 1):
                d = sub_dists[i]
                if d < 1e-9:
                    continue
                seg_dur = available_s * (d / total_d)
                segments.append(RouteSegment(
                    p1=tuple(path[i]), p2=tuple(path[i + 1]),
                    segment_type=SegmentType.deadhead,
                    cycle_index=cycle_index,
                    distance_m=d,
                    duration_s=max(seg_dur, 0.001),
                    energy_cost_wh=0.0, reagent_consumed_l=0.0,
                ))
