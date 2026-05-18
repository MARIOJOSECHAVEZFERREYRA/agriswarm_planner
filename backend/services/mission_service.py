"""Mission persistence, validation, and planning orchestration."""

import json
import logging
from json import JSONDecodeError

from fastapi import HTTPException
from shapely.geometry import Point, Polygon
from sqlalchemy.orm import Session

from backend.database import SessionLocal
from backend.models.mission_model import Mission, MissionStatus, Waypoint, WaypointType
from backend.schemas.mission import MissionCreate
from backend.services.mission_planner import DynamicMissionPlanner, StaticMissionPlanner


logger = logging.getLogger(__name__)

DEFAULT_DRONE_NAME = "DJI Agras T30"
MIN_FIELD_AREA_M2 = 100
MIN_OBSTACLE_AREA_M2 = 1


def validate_mission_request(payload: MissionCreate) -> None:
    field = payload.field
    exterior = [tuple(point) for point in field.coordinates]
    holes = [[tuple(point) for point in ring] for ring in field.obstacles]
    base_point = tuple(field.base_point)

    if len(exterior) < 3:
        raise HTTPException(status_code=422, detail="Field polygon requires at least 3 vertices.")

    ext_poly = _sanitize_polygon(Polygon(exterior))
    if ext_poly.is_empty:
        raise HTTPException(status_code=422, detail="Field polygon is empty or degenerate.")
    if ext_poly.area < MIN_FIELD_AREA_M2:
        raise HTTPException(
            status_code=422,
            detail=f"Field area ({ext_poly.area:.1f} m²) is below minimum ({MIN_FIELD_AREA_M2} m²).",
        )

    obstacle_polys = []
    for index, hole in enumerate(holes, start=1):
        if len(hole) < 3:
            raise HTTPException(status_code=422, detail=f"Obstacle {index} requires at least 3 vertices.")
        obs_poly = _sanitize_polygon(Polygon(hole))
        if obs_poly.is_empty:
            raise HTTPException(status_code=422, detail=f"Obstacle {index} is empty or degenerate.")
        if obs_poly.area < MIN_OBSTACLE_AREA_M2:
            raise HTTPException(
                status_code=422,
                detail=f"Obstacle {index} area ({obs_poly.area:.2f} m²) is below minimum ({MIN_OBSTACLE_AREA_M2} m²).",
            )
        if not ext_poly.contains(obs_poly):
            raise HTTPException(
                status_code=422,
                detail=f"Obstacle {index} is not fully contained within the field boundary.",
            )
        for prev_index, prev in enumerate(obstacle_polys, start=1):
            if obs_poly.intersects(prev):
                raise HTTPException(
                    status_code=422,
                    detail=f"Obstacles {prev_index} and {index} overlap or touch each other.",
                )
        obstacle_polys.append(obs_poly)

    full_poly = _sanitize_polygon(Polygon(shell=exterior, holes=holes))
    if full_poly.is_empty or full_poly.area < MIN_FIELD_AREA_M2:
        raise HTTPException(
            status_code=422,
            detail="Field geometry after subtracting obstacles is too small or degenerate.",
        )

    if full_poly.contains(Point(base_point)):
        raise HTTPException(
            status_code=422,
            detail="base_point must be outside the spray polygon or on its boundary.",
        )


def create_mission(db: Session, payload: MissionCreate) -> Mission:
    mission = Mission(
        name=payload.name,
        status=MissionStatus.pending,
        field_geojson=payload.field.model_dump_json(),
        spray_width=payload.spray_width,
        strategy=payload.strategy,
        drone_name=payload.drone_name,
        overrides_json=_build_overrides_json(payload),
    )
    db.add(mission)
    db.commit()
    db.refresh(mission)
    return mission


def compute_mission_by_id(mission_id: int) -> None:
    db = SessionLocal()
    try:
        mission = get_mission(db, mission_id)
        if mission is not None:
            compute_mission(db, mission)
    finally:
        db.close()


def compute_mission(db: Session, mission: Mission) -> Mission:
    mission.status = MissionStatus.running
    db.commit()

    try:
        field_data = deserialize_field(mission.field_geojson)
        planning_inputs = _build_planning_inputs(field_data)
        overrides, strategy_params = _load_overrides(mission)
        planner = _build_mission_planner(planning_inputs)

        result = planner.run_mission_planning(
            db=db,
            polygon_points=planning_inputs["polygon_points"],
            drone_name=mission.drone_name or DEFAULT_DRONE_NAME,
            overrides=overrides,
            base_point=planning_inputs["base_point"],
            strategy_name=mission.strategy,
            obstacle_polygons=planning_inputs["obstacle_polygons"],
            strategy_params=strategy_params,
        )
        _store_planning_result(db, mission, result)
    except Exception as exc:
        logger.exception("Mission planning failed for mission_id=%s", mission.id)
        mission.status = MissionStatus.failed
        mission.error_message = str(exc)

    db.commit()
    db.refresh(mission)
    return mission


def get_mission(db: Session, mission_id: int) -> Mission | None:
    return db.get(Mission, mission_id)


def list_missions(db: Session, skip: int = 0, limit: int = 50) -> list[Mission]:
    return db.query(Mission).order_by(Mission.created_at.desc()).offset(skip).limit(limit).all()


def deserialize_field(raw_field: str) -> dict:
    field_data = _parse_json(raw_field)
    if not isinstance(field_data, dict):
        raise ValueError("Mission field payload is missing or invalid.")
    return field_data


def deserialize_mission_cycles(raw_cycles: str | None) -> list[dict]:
    mission_cycles = _parse_json(raw_cycles)
    if not isinstance(mission_cycles, list):
        raise ValueError("Mission cycles are missing or invalid.")
    return mission_cycles


def _build_overrides_json(payload: MissionCreate) -> str | None:
    overrides = {}
    if payload.app_rate is not None:
        overrides["app_rate"] = payload.app_rate
    if payload.cruise_speed_ms is not None:
        overrides["speed"] = payload.cruise_speed_ms
    if payload.margin_m is not None:
        overrides["margin"] = payload.margin_m
    if payload.strategy_params:
        overrides["_strategy_params"] = payload.strategy_params
    return json.dumps(overrides) if overrides else None


def _sanitize_polygon(polygon: Polygon) -> Polygon:
    if polygon.is_valid:
        return polygon
    return polygon.buffer(0)


def _build_planning_inputs(field_data: dict) -> dict:
    raw_ugv = field_data.get("ugv_polyline")
    ugv_polyline = [tuple(point) for point in raw_ugv] if raw_ugv and len(raw_ugv) >= 2 else None
    obstacle_polygons = [[tuple(point) for point in ring] for ring in field_data.get("obstacles", [])]
    return {
        "polygon_points": [tuple(point) for point in field_data["coordinates"]],
        "obstacle_polygons": obstacle_polygons or None,
        "base_point": tuple(field_data["base_point"]),
        "ugv_polyline": ugv_polyline,
        "ugv_speed": float(field_data.get("ugv_speed", 2.0)),
        "ugv_t_service": float(field_data.get("ugv_t_service", 300.0)),
    }


def _build_mission_planner(planning_inputs: dict):
    if planning_inputs["ugv_polyline"]:
        return DynamicMissionPlanner(
            ugv_polyline=planning_inputs["ugv_polyline"],
            ugv_speed=planning_inputs["ugv_speed"],
            ugv_t_service=planning_inputs["ugv_t_service"],
        )
    return StaticMissionPlanner()


def _load_overrides(mission: Mission) -> tuple[dict, dict | None]:
    stored_overrides = _parse_json(mission.overrides_json) or {}
    strategy_params = stored_overrides.pop("_strategy_params", None)
    return {"swath": mission.spray_width, **stored_overrides}, strategy_params


def _store_planning_result(db: Session, mission: Mission, result: dict) -> None:
    mission_cycles = result.get("mission_cycles", [])
    best_path = result.get("best_path")
    safe_polygon = result.get("safe_polygon")

    mission.mission_cycles_json = json.dumps([_serialize_cycle(cycle) for cycle in mission_cycles])
    _replace_waypoints(db, mission, mission_cycles)
    mission.best_angle = result.get("best_angle")
    mission.n_cycles = len(mission_cycles)
    mission.total_distance = float(best_path.length) if best_path else None
    mission.coverage_area = float(safe_polygon.area) if safe_polygon else None
    mission.metrics_json = json.dumps(_build_metrics(result, safe_polygon))
    mission.status = MissionStatus.completed
    mission.error_message = None


def _parse_json(raw: str | None) -> dict | list | None:
    if not raw:
        return None
    try:
        return json.loads(raw)
    except (JSONDecodeError, TypeError):
        return None


def _serialize_point(point) -> list[float]:
    try:
        if hasattr(point, "__getitem__"):
            return [float(point[0]), float(point[1])]
        if hasattr(point, "x") and hasattr(point, "y"):
            return [float(point.x), float(point.y)]
    except (IndexError, AttributeError, TypeError):
        pass
    return [0.0, 0.0]


def _serialize_cycle(cycle: dict) -> dict:
    return {
        "segments": [
            {
                "p1": _serialize_point(seg.get("p1")),
                "p2": _serialize_point(seg.get("p2")),
                "spraying": bool(seg.get("spraying", False)),
                "segment_type": seg.get("segment_type"),
            }
            for seg in cycle.get("segments", [])
        ],
        "base_point": _serialize_point(cycle.get("base_point")),
        "swath_width": float(cycle.get("swath_width", 0.0)),
        **({"rv_wait_s": float(cycle["rv_wait_s"])} if "rv_wait_s" in cycle else {}),
    }


def _replace_waypoints(db: Session, mission: Mission, mission_cycles: list[dict]) -> None:
    db.query(Waypoint).filter(Waypoint.mission_id == mission.id).delete()

    waypoints = []
    seq = 0
    prev_pt = None

    for cycle_index, cycle in enumerate(mission_cycles):
        for seg in cycle.get("segments", []):
            p1 = (float(seg["p1"][0]), float(seg["p1"][1]))
            p2 = (float(seg["p2"][0]), float(seg["p2"][1]))
            waypoint_type = _map_segment_type(seg.get("segment_type"))

            same_pt = prev_pt == p1
            can_merge = (
                same_pt
                and waypoints
                and waypoints[-1].waypoint_type != WaypointType.base
                and waypoints[-1].cycle_index == cycle_index
            )

            if can_merge:
                waypoints[-1].waypoint_type = waypoint_type
            else:
                waypoints.append(
                    Waypoint(
                        mission_id=mission.id,
                        sequence=seq,
                        x=p1[0],
                        y=p1[1],
                        waypoint_type=waypoint_type,
                        cycle_index=cycle_index,
                    )
                )
                seq += 1
                prev_pt = p1

            if prev_pt != p2:
                waypoints.append(
                    Waypoint(
                        mission_id=mission.id,
                        sequence=seq,
                        x=p2[0],
                        y=p2[1],
                        waypoint_type=waypoint_type,
                        cycle_index=cycle_index,
                    )
                )
                seq += 1
                prev_pt = p2

        cycle_base = tuple(cycle.get("base_point", [0.0, 0.0]))
        if prev_pt == cycle_base and waypoints:
            waypoints[-1].waypoint_type = WaypointType.base
            waypoints[-1].cycle_index = cycle_index
        else:
            waypoints.append(
                Waypoint(
                    mission_id=mission.id,
                    sequence=seq,
                    x=float(cycle_base[0]),
                    y=float(cycle_base[1]),
                    waypoint_type=WaypointType.base,
                    cycle_index=cycle_index,
                )
            )
            seq += 1
        prev_pt = cycle_base

    db.add_all(waypoints)


def _build_metrics(result: dict, safe_polygon) -> dict:
    metrics = dict(result.get("metrics") or {})
    if safe_polygon and not safe_polygon.is_empty:
        try:
            metrics["_safe_polygon"] = [list(point) for point in safe_polygon.exterior.coords]
        except (AttributeError, TypeError, ValueError):
            pass
    if result.get("rv_infeasible"):
        metrics["rv_warning"] = "infeasible_partial_mission"
        metrics["rv_warning_reason"] = result.get("rv_infeasible_reason")
    return metrics


def _map_segment_type(explicit_type):
    if explicit_type == "sweep":
        return WaypointType.sweep
    if explicit_type == "ferry":
        return WaypointType.ferry
    if explicit_type == "deadhead":
        return WaypointType.deadhead
    raise ValueError(f"Unknown or missing segment_type: {explicit_type}")
