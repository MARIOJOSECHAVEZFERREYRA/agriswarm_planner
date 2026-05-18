import asyncio
from contextlib import suppress

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect
from pydantic import ValidationError
from shapely.geometry import Polygon
from sqlalchemy.orm import Session

from backend.database import get_db
from backend.models.drone_model import Drone
from backend.models.mission_model import MissionStatus
from backend.schemas.simulation import SimulationConfig
from backend.services import mission_service, simulation_service
from backend.services.route_builder_service import UAVRouteBuilder, UGVRouteBuilder

router = APIRouter(tags=["simulation"])


def _build_simulation_routes(mission, drone):
    field_data = mission_service.deserialize_field(mission.field_geojson)
    mission_cycles = mission_service.deserialize_mission_cycles(mission.mission_cycles_json)
    if not mission_cycles:
        raise ValueError("Mission has no cycles to simulate.")

    work_polygon = Polygon(field_data["coordinates"])
    raw_ugv_route = field_data.get("ugv_polyline")
    is_mobile_mode = raw_ugv_route is not None and len(raw_ugv_route) >= 2
    ugv_speed = float(field_data.get("ugv_speed", 2.0))
    ugv_service_time_s = float(field_data.get("ugv_t_service", 300.0))

    uav_route = UAVRouteBuilder(drone, work_polygon).build(
        mission_cycles,
        service_duration_s=ugv_service_time_s if is_mobile_mode else None,
    )
    ugv_builder = UGVRouteBuilder()
    if is_mobile_mode:
        ugv_route = ugv_builder.build_mobile(
            mission_cycles=mission_cycles,
            uav_route=uav_route,
            ugv_polyline=raw_ugv_route,
            ugv_speed=ugv_speed,
            ugv_t_service=ugv_service_time_s,
        )
    else:
        ugv_route = ugv_builder.build_static(mission_cycles, uav_route.total_duration_s)
    return uav_route, ugv_route


@router.websocket("/simulation/{mission_id}")
async def simulation_ws(
    mission_id: int,
    ws: WebSocket,
    db: Session = Depends(get_db),
):
    mission = mission_service.get_mission(db, mission_id)
    if not mission or mission.status != MissionStatus.completed:
        await ws.close(code=1008)
        return

    drone = db.query(Drone).filter(Drone.name == mission.drone_name).first()
    if not drone:
        await ws.close(code=1011)
        return

    try:
        uav_route, ugv_route = _build_simulation_routes(mission, drone)
    except (KeyError, TypeError, ValueError):
        await ws.close(code=1011)
        return

    await ws.accept()
    sim_state = simulation_service.SimulationState()

    async def recv_config():
        while True:
            try:
                data = await ws.receive_text()
            except WebSocketDisconnect:
                return

            try:
                cfg = SimulationConfig.model_validate_json(data)
            except ValidationError:
                continue

            sim_state.playback_speed = cfg.playback_speed

    recv_task = asyncio.create_task(recv_config())

    try:
        async for frame in simulation_service.stream_simulation(
            uav_route=uav_route,
            ugv_route=ugv_route,
            drone=drone,
            state=sim_state,
            interval_ms=200,
        ):
            await ws.send_text(frame.model_dump_json())
    except WebSocketDisconnect:
        return
    finally:
        recv_task.cancel()
        with suppress(asyncio.CancelledError):
            await recv_task
