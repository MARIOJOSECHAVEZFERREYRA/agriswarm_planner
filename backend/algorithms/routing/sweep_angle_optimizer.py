import logging
import math
import random
import numpy as np
from shapely import affinity
from shapely.geometry import Polygon
from shapely.ops import unary_union
from ..coverage.decomposition import ConcaveDecomposer
from ..coverage.path_assembler import PathAssembler
from ..simulation.mission_simulator import simulate_mission_with_rendezvous
logger = logging.getLogger(__name__)
def build_obstacle_union(polygon: Polygon):
    if polygon.interiors:
        return unary_union([Polygon(h.coords) for h in polygon.interiors])
    return None
class SweepAngleOptimizer:
    def __init__(self, planner, pop_size=200, generations=300, crossover_rate=0.4, mutation_rate=0.05, mutation_range=15, min_diversity=20, early_stopping_patience=50, energy_model=None, rendezvous_planner=None, w_rv=0.5, cycle_penalty_s=0.0,):
        self.planner = planner
        self.pop_size = pop_size
        self.generations = generations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.mutation_range = mutation_range
        self.min_diversity = min_diversity
        self.early_stopping_patience = early_stopping_patience
        self.energy_model = energy_model
        self.rendezvous_planner = rendezvous_planner
        self.w_rv = float(w_rv)
        self.cycle_penalty_s = float(cycle_penalty_s)
        self._rv_enabled = energy_model is not None and rendezvous_planner is not None
        self._static_deadhead_enabled = energy_model is not None and rendezvous_planner is None
    _SERVICE_DISCOUNT = 0.3
    @staticmethod
    def _pt_to_polyline_dist(px, py, polyline):
        best = float('inf')
        for i in range(len(polyline) - 1):
            ax, ay = polyline[i]
            bx, by = polyline[i + 1]
            dx, dy = bx - ax, by - ay
            sl2 = dx * dx + dy * dy
            if sl2 < 1e-18:
                continue
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / sl2))
            d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
            if d < best:
                best = d
        return best
    def _avg_min_endpoint_polyline_dist(self, route_segments):
        poly = self.rendezvous_planner.ugv_polyline
        total, n = 0.0, 0
        for seg in route_segments:
            if seg.get('segment_type') != 'sweep':
                continue
            path = seg.get('path', [])
            if len(path) < 2:
                continue
            d1 = self._pt_to_polyline_dist(
                float(path[0][0]), float(path[0][1]), poly)
            d2 = self._pt_to_polyline_dist(
                float(path[-1][0]), float(path[-1][1]), poly)
            total += min(d1, d2)
            n += 1
        return total / n if n > 0 else 0.0
    def _dynamic_fitness(self, rv_time, rv_count, route_segments):
        t_service = self.rendezvous_planner.t_service
        v_uav = float(self.energy_model.drone.speed_max_ms)
        flight_only = rv_time - rv_count * t_service
        service_cost = rv_count * t_service * self._SERVICE_DISCOUNT
        avg_min_dist = self._avg_min_endpoint_polyline_dist(route_segments)
        alignment = self.w_rv * rv_count * avg_min_dist / v_uav
        return flight_only + service_cost + alignment
    def _estimate_static_deadhead(self, route_segments, base_point):
        em = self.energy_model
        drone = em.drone
        bx, by = float(base_point[0]), float(base_point[1])
        e_rem = em.usable_energy_wh()
        q_rem = float(drone.mass_tank_full_kg)
        deadhead_total = 0.0
        cycle_empty = True
        cycle_breaks = 0
        uav_x, uav_y = bx, by
        if route_segments:
            first_path = route_segments[0].get('path', [])
            if first_path:
                fx, fy = float(first_path[0][0]), float(first_path[0][1])
                d_entry = math.hypot(fx - bx, fy - by)
                e_rem = max(0.0, e_rem - em.energy_transit(d_entry, q_rem))
                uav_x, uav_y = fx, fy
        for seg in route_segments:
            seg_type = seg.get('segment_type', 'ferry')
            dist = float(seg.get('distance_m', 0.0))
            path = seg.get('path', [])
            if dist < 1e-9 or len(path) < 2:
                continue
            p1x, p1y = float(path[0][0]), float(path[0][1])
            p2x, p2y = float(path[-1][0]), float(path[-1][1])
            if seg_type == 'sweep':
                e_step = em.energy_straight(dist, q_rem)
                q_step = em.reagent_consumed(dist)
            else:
                e_step = em.energy_transit(dist, q_rem)
                q_step = 0.0
            d_after_to_base = math.hypot(p2x - bx, p2y - by)
            can_do = em.feasible_after_segment_static(
                e_rem, q_rem, e_step, q_step, d_after_to_base
            )
            if can_do or cycle_empty:
                e_rem -= e_step
                q_rem = max(0.0, q_rem - q_step)
                uav_x, uav_y = p2x, p2y
                cycle_empty = False
            else:
                d_back = math.hypot(uav_x - bx, uav_y - by)
                d_fwd = math.hypot(p1x - bx, p1y - by)
                deadhead_total += d_back + d_fwd
                cycle_breaks += 1
                e_rem = em.usable_energy_wh()
                q_rem = float(drone.mass_tank_full_kg)
                e_rem = max(0.0, e_rem - em.energy_transit(d_fwd, q_rem))
                if seg_type == 'sweep':
                    e_step = em.energy_straight(dist, q_rem)
                    q_step = em.reagent_consumed(dist)
                else:
                    e_step = em.energy_transit(dist, q_rem)
                    q_step = 0.0
                e_rem -= e_step
                q_rem = max(0.0, q_rem - q_step)
                uav_x, uav_y = p2x, p2y
                cycle_empty = False
        n_cycles = cycle_breaks + 1 if route_segments else 0
        return deadhead_total, n_cycles
    def _compute_mission_time(self, spray_m, ferry_m, deadhead_m, n_cycles, n_turns=0):
        if self.energy_model is not None:
            drone = self.energy_model.drone
            v_cruise = float(getattr(drone, 'speed_cruise_ms', 5.0))
            v_max = float(getattr(drone, 'speed_max_ms', v_cruise * 1.5))
            turn_s = float(getattr(drone, 'turn_duration_s', 10.0))
        else:
            v_cruise = 5.0
            v_max = 10.0
            turn_s = 10.0
        v_cruise = max(v_cruise, 0.1)
        v_max = max(v_max, 0.1)
        flight_time_s = (
            spray_m / v_cruise +
            ferry_m / v_max +
            deadhead_m / v_max +
            n_turns * turn_s
        )
        cycle_penalty = self.cycle_penalty_s * max(n_cycles - 1, 0)
        mission_time_s = flight_time_s + cycle_penalty
        return mission_time_s, flight_time_s
    def _plan_cells(self, polygon, angle_deg, obstacle_union):
        sub_polygons = ConcaveDecomposer.decompose(polygon, angle_deg,
                                                    channel_width=self.planner.spray_width,
                                                    min_swath=self.planner.spray_width)
        rotation_origin = polygon.centroid
        rotated_whole = affinity.rotate(polygon, -angle_deg, origin=rotation_origin)
        global_y_origin = rotated_whole.bounds[1]
        all_sweep_segments = []
        spray_m = 0.0
        s_prime = 0.0
        turns = 0
        for cell_id, sub_poly in enumerate(sub_polygons):
            planner_result = self.planner.generate_path(
                sub_poly,
                angle_deg,
                global_y_origin=global_y_origin,
                rotation_origin=rotation_origin,
                obstacles=obstacle_union,
            )
            sweep_segments = planner_result.get("sweep_segments", [])
            metrics = planner_result.get("metrics", {})
            if sweep_segments:
                for s in sweep_segments:
                    s['cell_id'] = cell_id
                all_sweep_segments.extend(sweep_segments)
            spray_m += float(metrics.get("spray_distance_m", 0.0))
            s_prime += float(metrics.get("coverage_area_m2", 0.0))
            turns += int(metrics.get("turn_count", 0))
        return sub_polygons, all_sweep_segments, spray_m, s_prime, turns
    def _make_assembler(self, sub_polygons, polygon, base_point, sequencer_mode):
        ugv_poly = self.rendezvous_planner.ugv_polyline if self._rv_enabled else None
        return PathAssembler(
            sub_polygons,
            original_polygon=polygon,
            sequencer_mode=sequencer_mode,
            base_point=base_point,
            ugv_polyline=ugv_poly,
        )
    def _compute_deadhead(self, assembler, route_segments, base_point):
        deadhead_m = 0.0
        n_cycles = 1
        if not self._rv_enabled and base_point is not None and route_segments:
            bp = (float(base_point[0]), float(base_point[1]))
            _, d_entry = assembler.find_connection(bp, route_segments[0]["path"][0])
            _, d_exit = assembler.find_connection(route_segments[-1]["path"][-1], bp)
            deadhead_m += d_entry + d_exit
            if self._static_deadhead_enabled:
                intermediate, n_cycles = self._estimate_static_deadhead(route_segments, bp)
                deadhead_m += intermediate
        return deadhead_m, n_cycles
    def _run_rendezvous(self, route_segments, assembler):
        if not (self._rv_enabled and route_segments):
            return None
        sim = simulate_mission_with_rendezvous(
            route_segments=route_segments,
            energy_model=self.energy_model,
            rendezvous_planner=self.rendezvous_planner,
            assembler=assembler,
        )
        return {
            'rv_feasible': sim['feasible'],
            'rv_wait': sim['total_wait_uav'],
            'rv_time': sim['total_time'],
            'rv_count': sim['n_rendezvous'],
        }
    def _evaluate_angle(self, angle_deg: int, polygon: Polygon, obstacle_union=None, base_point=None,
    ) -> dict:
        sub_polygons, all_sweep_segments, spray_m, s_prime, turns = self._plan_cells(
            polygon, angle_deg, obstacle_union)
        assembler = self._make_assembler(sub_polygons, polygon, base_point, 'fast')
        assembly = assembler.assemble_connected(all_sweep_segments)
        route_segments = assembly.get("route_segments", [])
        route_distances = assembly.get("distances", {})
        ferry_m = float(route_distances.get("ferry_m", 0.0))
        deadhead_m, n_cycles = self._compute_deadhead(assembler, route_segments, base_point)
        mission_time_s, flight_time_s = self._compute_mission_time(
            spray_m, ferry_m, deadhead_m, n_cycles, n_turns=turns)
        total_l = mission_time_s
        rv = self._run_rendezvous(route_segments, assembler)
        if rv and rv['rv_feasible']:
            total_l = self._dynamic_fitness(rv['rv_time'], rv['rv_count'], route_segments)
        return {
            "angle": angle_deg,
            "l": total_l,
            "mission_time_s": mission_time_s,
            "flight_time_s": flight_time_s,
            "n_cycles": n_cycles,
            "spray_m": float(spray_m),
            "ferry_m": ferry_m,
            "deadhead_m": deadhead_m,
            "s_prime": s_prime,
            "combined_path": assembly.get("combined_path", []),
            "route_segments": route_segments,
            "planner_metrics": {
                "spray_distance_m": float(spray_m),
                "coverage_area_m2": float(s_prime),
                "turn_count": int(turns),
            },
            "route_distances": {
                "sweep_m": float(route_distances.get("sweep_m", 0.0)),
                "ferry_m": ferry_m,
                "total_m": float(route_distances.get("total_m", 0.0)),
            },
            "rv_feasible": rv['rv_feasible'] if rv else True,
            "rv_wait": rv['rv_wait'] if rv else 0.0,
            "rv_time": rv['rv_time'] if rv else 0.0,
            "rv_count": rv['rv_count'] if rv else 0,
        }
    @staticmethod
    def _compute_fitness(l_norm, s_prime, target_area_S, rv_penalty_norm=0.0, w_rv=0.0):
        if target_area_S <= 0:
            return 1e-10
        extra_coverage = abs(s_prime - target_area_S) / target_area_S
        denominator = l_norm + extra_coverage + w_rv * rv_penalty_norm
        if denominator < 1e-12:
            return 1e10
        return 1.0 / denominator
    @staticmethod
    def _tournament_selection(population: list[int], fitness_values: list[float], k: int = 3) -> int:
        indices = random.sample(range(len(population)), k)
        best = max(indices, key=lambda i: fitness_values[i])
        return population[best]
    @staticmethod
    def _blend_crossover(p1: int, p2: int):
        diff = (p2 - p1 + 90) % 180 - 90
        alpha = random.random()
        offset = round(alpha * diff)
        c1 = (p1 + offset) % 180
        c2 = (p1 - diff + offset) % 180
        return c1, c2
    def _mutate(self, angle: int) -> int:
        if random.random() < self.mutation_rate:
            delta = random.randint(-self.mutation_range, self.mutation_range)
            return (angle + delta) % 180
        return angle
    def _evaluate_population(self, population: list[int], polygon: Polygon, target_area_S: float, cache: dict, obstacle_union=None, base_point=None,
    ):
        raw_metrics = []
        for angle in population:
            if angle not in cache:
                cache[angle] = self._evaluate_angle(
                    angle,
                    polygon,
                    obstacle_union=obstacle_union,
                    base_point=base_point,
                )
            raw_metrics.append(cache[angle])
        all_l = np.array([m["l"] for m in raw_metrics], dtype=float)
        l2_norm = np.sqrt(np.sum(all_l ** 2))
        if l2_norm < 1e-12:
            l2_norm = 1.0
        l_norms = all_l / l2_norm
        if self._rv_enabled:
            rv_waits = []
            for m in raw_metrics:
                if m.get('rv_feasible', True):
                    rv_waits.append(m.get('rv_wait', 0.0))
                else:
                    rv_waits.append(1e9)
            all_rv_wait = np.array(rv_waits, dtype=float)
            rv_l2 = np.sqrt(np.sum(all_rv_wait ** 2))
            if rv_l2 < 1e-12:
                rv_l2 = 1.0
            rv_norms = all_rv_wait / rv_l2
            fitness_values = []
            for i in range(len(raw_metrics)):
                if not raw_metrics[i].get('rv_feasible', True):
                    fitness_values.append(1e-10)
                else:
                    f = self._compute_fitness(
                        l_norms[i],
                        raw_metrics[i]["s_prime"],
                        target_area_S,
                        rv_penalty_norm=rv_norms[i],
                        w_rv=self.w_rv,
                    )
                    fitness_values.append(f)
        else:
            fitness_values = [
                self._compute_fitness(l_norms[i], raw_metrics[i]["s_prime"], target_area_S)
                for i in range(len(raw_metrics))
            ]
        return raw_metrics, fitness_values, all_l
    def optimize(self, polygon: Polygon, base_point=None) -> dict:
        random.seed(42)
        target_area_S = polygon.area
        eval_cache = {}
        obstacle_union = build_obstacle_union(polygon)
        population = [random.randint(0, 179) for _ in range(self.pop_size)]
        best_solution = None
        best_fitness = -1.0
        no_improvement_count = 0
        gen_stats = []
        for gen in range(self.generations):
            raw_metrics, fitness_values, all_l = self._evaluate_population(
                population,
                polygon,
                target_area_S,
                eval_cache,
                obstacle_union=obstacle_union,
                base_point=base_point,
            )
            gen_stats.append({
                "gen": gen + 1,
                "mean_fitness": float(np.mean(fitness_values)),
                "mean_angle": float(np.mean(population)),
                "mean_l": float(np.mean(all_l)),
            })
            prev_best_fitness = best_fitness
            for i, f in enumerate(fitness_values):
                if f > best_fitness:
                    best_fitness = f
                    m = raw_metrics[i]
                    extra_cov = abs(m["s_prime"] - target_area_S) / target_area_S * 100.0
                    best_solution = {
                        "angle": m["angle"],
                        "fitness": f,
                        "l": m["l"],
                        "s_prime": m["s_prime"],
                        "extra_coverage_pct": extra_cov,
                        "route_segments": m.get("route_segments", []),
                        "combined_path": m.get("combined_path", []),
                        "planner_metrics": m.get("planner_metrics", {}),
                        "route_distances": m.get("route_distances", {}),
                        "rv_feasible": m.get("rv_feasible", True),
                        "rv_wait": m.get("rv_wait", 0.0),
                        "rv_time": m.get("rv_time", 0.0),
                        "rv_count": m.get("rv_count", 0),
                    }
            selected = [
                self._tournament_selection(population, fitness_values, k=3)
                for _ in range(self.pop_size)
            ]
            kids = []
            i = 0
            while i < len(selected) - 1:
                p1 = selected[i]
                p2 = selected[i + 1]
                if random.random() < self.crossover_rate:
                    c1, c2 = self._blend_crossover(p1, p2)
                else:
                    c1, c2 = p1, p2
                kids.append(self._mutate(c1))
                kids.append(self._mutate(c2))
                i += 2
            if i < len(selected):
                kids.append(self._mutate(selected[i]))
            elite = population[int(np.argmax(fitness_values))]
            population = kids[:self.pop_size]
            population[0] = elite
            if len(set(population)) < self.min_diversity:
                n_inject = self.pop_size // 5
                inject_indices = random.sample(range(1, self.pop_size), n_inject)
                for idx in inject_indices:
                    population[idx] = random.randint(0, 179)
            if best_fitness > prev_best_fitness:
                no_improvement_count = 0
            else:
                no_improvement_count += 1
            if no_improvement_count >= self.early_stopping_patience:
                logger.info("SweepAngleOptimizer early stop at generation %s", gen + 1)
                break
            if (gen + 1) % 25 == 0 and best_solution is not None:
                s = gen_stats[-1]
                logger.info(
                    "SweepAngleOptimizer gen %s/%s | best %.4f @ %s° | mean fit %.4f angle %.1f° L %.0fm",
                    gen + 1,
                    self.generations,
                    best_fitness,
                    best_solution["angle"],
                    s["mean_fitness"],
                    s["mean_angle"],
                    s["mean_l"],
                )
        all_cached = list(eval_cache.values())
        if not all_cached:
            raise ValueError("SweepAngleOptimizer failed to find a valid solution.")
        all_l_final = np.array([m["l"] for m in all_cached], dtype=float)
        l2_final = float(np.sqrt(np.sum(all_l_final ** 2)))
        if l2_final < 1e-12:
            l2_final = 1.0
        if self._rv_enabled:
            rv_waits_final = [
                m.get("rv_wait", 0.0) if m.get("rv_feasible", True) else 1e9
                for m in all_cached
            ]
            rv_l2_final = float(np.sqrt(np.sum(np.array(rv_waits_final, dtype=float) ** 2)))
            if rv_l2_final < 1e-12:
                rv_l2_final = 1.0
        else:
            rv_waits_final = [0.0] * len(all_cached)
            rv_l2_final = 1.0
        best_abs_fitness = -1.0
        best_solution = None
        for idx, m in enumerate(all_cached):
            if self._rv_enabled and not m.get("rv_feasible", True):
                continue
            l_norm = m["l"] / l2_final
            rv_norm = rv_waits_final[idx] / rv_l2_final if self._rv_enabled else 0.0
            f = self._compute_fitness(
                l_norm, m["s_prime"], target_area_S,
                rv_penalty_norm=rv_norm, w_rv=self.w_rv,
            )
            if f > best_abs_fitness:
                best_abs_fitness = f
                extra_cov = abs(m["s_prime"] - target_area_S) / target_area_S * 100.0
                best_solution = dict(m)
                best_solution["fitness"] = f
                best_solution["extra_coverage_pct"] = extra_cov
        if best_solution is None:
            if self._rv_enabled:
                raise ValueError(
                    "SweepAngleOptimizer: all evaluated angles produced infeasible "
                    "rendezvous missions. The field may be too large for the drone's "
                    "range, or the UGV polyline is unreachable."
                )
            raise ValueError("SweepAngleOptimizer failed to find a valid solution.")
        best_solution = self._reassemble_full(
            best_solution, polygon, obstacle_union, base_point,
        )
        mission_min = best_solution['l'] / 60.0
        logger.info(
            "SweepAngleOptimizer done | angle=%s° fitness=%.4f mission=%.1fmin cycles=%s spray=%.0fm ferry=%.0fm deadhead=%.0fm coverage+=%.2f%%",
            best_solution["angle"],
            best_abs_fitness,
            mission_min,
            best_solution.get("n_cycles", "?"),
            best_solution.get("spray_m", 0),
            best_solution.get("ferry_m", 0),
            best_solution.get("deadhead_m", 0),
            best_solution["extra_coverage_pct"],
        )
        best_solution["gen_stats"] = gen_stats
        return best_solution
    def _reassemble_full(self, best_solution, polygon, obstacle_union, base_point):
        angle = int(best_solution["angle"])
        sub_polygons, all_sweep_segments, _, _, _ = self._plan_cells(
            polygon, angle, obstacle_union)
        assembler = self._make_assembler(sub_polygons, polygon, base_point, 'full')
        assembly = assembler.assemble_connected(all_sweep_segments)
        route_segments = assembly.get("route_segments", [])
        best_solution = dict(best_solution)
        best_solution["route_segments"] = route_segments
        best_solution["combined_path"] = assembly.get("combined_path", [])
        best_solution["route_distances"] = assembly.get("distances", {})
        spray_m = float(best_solution.get("spray_m", 0.0))
        ferry_m = float(assembly.get("distances", {}).get("ferry_m", 0.0))
        deadhead_m, n_cycles = self._compute_deadhead(assembler, route_segments, base_point)
        turns = int(best_solution.get('planner_metrics', {}).get('turn_count', 0))
        mission_time_s, flight_time_s = self._compute_mission_time(
            spray_m, ferry_m, deadhead_m, n_cycles, n_turns=turns)
        best_solution["ferry_m"] = ferry_m
        best_solution["deadhead_m"] = deadhead_m
        best_solution["n_cycles"] = n_cycles
        best_solution["mission_time_s"] = mission_time_s
        best_solution["flight_time_s"] = flight_time_s
        best_solution["l"] = mission_time_s
        rv = self._run_rendezvous(route_segments, assembler)
        if rv:
            best_solution['rv_feasible'] = rv['rv_feasible']
            best_solution['rv_wait'] = rv['rv_wait']
            best_solution['rv_time'] = rv['rv_time']
            best_solution['rv_count'] = rv['rv_count']
            if rv['rv_feasible']:
                best_solution['l'] = self._dynamic_fitness(
                    rv['rv_time'], rv['rv_count'], route_segments)
        return best_solution
