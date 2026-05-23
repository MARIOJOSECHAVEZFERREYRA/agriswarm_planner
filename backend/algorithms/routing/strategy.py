from abc import ABC, abstractmethod

from .sweep_angle_optimizer import SweepAngleOptimizer, build_obstacle_union
from .grid_search_optimizer import GridSearchOptimizer
from ..coverage.path_planner import BoustrophedonPlanner


class MissionPlannerStrategy(ABC):
    @abstractmethod
    def optimize(
        self,
        polygon,
        swath_width,
        base_point=None,
        energy_model=None,
        rendezvous_planner=None,
        w_rv=0.5,
    ):
        raise NotImplementedError


class GeneticStrategy(MissionPlannerStrategy):
    def optimize(self, polygon, swath_width, base_point=None,
                 energy_model=None, rendezvous_planner=None, w_rv=0.5):
        planner = BoustrophedonPlanner(spray_width=swath_width)

        num_vertices = len(list(polygon.exterior.coords))
        poly_area = polygon.area

        if num_vertices <= 8 and poly_area <= 50000:
            params = {"pop_size": 200, "generations": 300}
        elif num_vertices <= 15 and poly_area <= 200000:
            params = {"pop_size": 150, "generations": 200}
        else:
            params = {"pop_size": 100, "generations": 150}

        optimizer = SweepAngleOptimizer(
            planner,
            pop_size=params["pop_size"],
            generations=params["generations"],
            early_stopping_patience=50,
            energy_model=energy_model,
            rendezvous_planner=rendezvous_planner,
            w_rv=w_rv,
        )

        best_solution = optimizer.optimize(polygon, base_point=base_point)

        return {
            "angle": best_solution.get("angle", 0.0),
            "route_segments": best_solution.get("route_segments", []),
            "combined_path": best_solution.get("combined_path", []),
            "planner_metrics": best_solution.get("planner_metrics", {}),
            "route_distances": best_solution.get("route_distances", {}),
            "rv_count": best_solution.get("rv_count", 0),
            "rv_wait": best_solution.get("rv_wait", 0.0),
            "metrics": {
                "fitness": best_solution.get("fitness"),
                "l": best_solution.get("l"),
                "s_prime": best_solution.get("s_prime"),
                "extra_coverage_pct": best_solution.get("extra_coverage_pct"),
            },
            "gen_stats": best_solution.get("gen_stats", []),
        }


class GridSearchStrategy(MissionPlannerStrategy):
    """Exhaustive integer-angle scan. Default strategy.

    Brute force over 180 integer angles: faster and optimal by
    construction for this 1D search space.
    """

    def __init__(self, angle_step: int = 1):
        self.angle_step = int(angle_step)

    def optimize(self, polygon, swath_width, base_point=None,
                 energy_model=None, rendezvous_planner=None, w_rv=0.5):
        planner = BoustrophedonPlanner(spray_width=swath_width)

        optimizer = GridSearchOptimizer(
            planner,
            angle_step=self.angle_step,
            energy_model=energy_model,
            rendezvous_planner=rendezvous_planner,
            w_rv=w_rv,
        )

        best = optimizer.optimize(polygon, base_point=base_point)

        return {
            "angle": best.get("angle", 0.0),
            "route_segments": best.get("route_segments", []),
            "combined_path": best.get("combined_path", []),
            "planner_metrics": best.get("planner_metrics", {}),
            "route_distances": best.get("route_distances", {}),
            "rv_count": best.get("rv_count", 0),
            "rv_wait": best.get("rv_wait", 0.0),
            "metrics": {
                "fitness": best.get("fitness"),
                "l": best.get("l"),
                "s_prime": best.get("s_prime"),
                "extra_coverage_pct": best.get("extra_coverage_pct"),
            },
            "gen_stats": best.get("gen_stats", []),
        }


class FixedAngleStrategy(MissionPlannerStrategy):
    """Evaluate a single fixed sweep angle without running a search."""

    def __init__(self, angle: float):
        self.angle = int(round(float(angle))) % 180

    def optimize(self, polygon, swath_width, base_point=None,
                 energy_model=None, rendezvous_planner=None, w_rv=0.5):
        planner = BoustrophedonPlanner(spray_width=swath_width)

        # Reuse the GA optimizer's evaluation pipeline (decompose, sweep,
        # assemble, segment, simulate) by instantiating it with trivial GA
        # parameters and invoking the per-angle evaluator directly.
        optimizer = SweepAngleOptimizer(
            planner,
            pop_size=1, generations=1, early_stopping_patience=1,
            energy_model=energy_model,
            rendezvous_planner=rendezvous_planner,
            w_rv=w_rv,
        )

        obstacle_union = build_obstacle_union(polygon)

        result = optimizer._evaluate_angle(
            self.angle, polygon,
            obstacle_union=obstacle_union,
            base_point=base_point,
        )

        result["fitness"] = 1.0 / result["l"] if result["l"] > 0 else 0.0
        target_area = polygon.area
        result["extra_coverage_pct"] = (
            abs(result.get("s_prime", 0.0) - target_area) / target_area * 100.0
            if target_area > 0 else 0.0
        )

        result = optimizer._reassemble_full(
            result, polygon, obstacle_union, base_point,
        )

        return {
            "angle": result.get("angle", self.angle),
            "route_segments": result.get("route_segments", []),
            "combined_path": result.get("combined_path", []),
            "planner_metrics": result.get("planner_metrics", {}),
            "route_distances": result.get("route_distances", {}),
            "rv_count": result.get("rv_count", 0),
            "rv_wait": result.get("rv_wait", 0.0),
            "metrics": {
                "fitness": result.get("fitness"),
                "l": result.get("l"),
                "s_prime": result.get("s_prime"),
                "extra_coverage_pct": result.get("extra_coverage_pct"),
            },
            "gen_stats": [],
        }


class StrategyFactory:
    @staticmethod
    def get_strategy(name: str, **params) -> MissionPlannerStrategy:
        key = name.lower()
        if key == "grid":
            return GridSearchStrategy()
        if key == "genetic":
            return GeneticStrategy()
        if key == "fixed":
            angle = params.get("angle")
            if angle is None:
                raise ValueError("FixedAngleStrategy requires 'angle' parameter")
            return FixedAngleStrategy(angle=angle)
        raise ValueError(f"Unknown strategy: {name}")
