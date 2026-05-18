from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session

from backend.database import get_db
from backend.models.drone_model import Drone
from backend.schemas.drone import DroneDefaults, DroneDetail, DroneSummary

router = APIRouter(prefix="/drones", tags=["drones"])


def _drone_or_404(name: str, db: Session) -> Drone:
    drone = db.query(Drone).filter(Drone.name == name).first()
    if not drone:
        raise HTTPException(status_code=404, detail="Drone not found")
    return drone


def _serialize_drone_summary(drone: Drone) -> DroneSummary:
    return DroneSummary(
        name=drone.name,
        num_rotors=drone.num_rotors,
        default_swath=drone.spray_swath_max_m,
        speed_cruise_ms=drone.speed_cruise_ms,
        speed_max_ms=drone.speed_max_ms,
        spray_flow_rate_lpm=drone.spray_flow_rate_lpm,
        battery_capacity_wh=drone.battery_capacity_wh,
        battery_reserve_pct=drone.battery_reserve_pct,
        service_time_s=drone.service_time_s,
    )


def _serialize_drone_defaults(drone: Drone) -> DroneDefaults:
    return DroneDefaults(
        swath_m=drone.spray_swath_max_m,
        spray_swath_min_m=drone.spray_swath_min_m,
        spray_swath_max_m=drone.spray_swath_max_m,
        margin_m=round(drone.spray_swath_max_m / 2.0, 2),
        app_rate_l_ha=drone.app_rate_default_l_ha,
        app_rate_min_l_ha=drone.app_rate_min_l_ha,
        app_rate_max_l_ha=drone.app_rate_max_l_ha,
        speed_ms=drone.speed_cruise_ms,
        speed_min_ms=drone.speed_cruise_ms,
        speed_max_ms=drone.speed_max_ms,
    )


def _serialize_drone_detail(drone: Drone) -> DroneDetail:
    mass_takeoff_max_kg = drone.mass_empty_kg + drone.mass_battery_kg + drone.mass_tank_full_kg
    return DroneDetail(
        id=drone.id,
        name=drone.name,
        num_rotors=drone.num_rotors,
        mass_empty_kg=drone.mass_empty_kg,
        mass_battery_kg=drone.mass_battery_kg,
        mass_tank_full_kg=drone.mass_tank_full_kg,
        mass_takeoff_max_kg=mass_takeoff_max_kg,
        battery_capacity_wh=drone.battery_capacity_wh,
        battery_voltage_v=drone.battery_voltage_v,
        battery_reserve_pct=drone.battery_reserve_pct,
        battery_charge_time_min=drone.battery_charge_time_min,
        power_hover_empty_w=drone.power_hover_empty_w,
        power_hover_full_w=drone.power_hover_full_w,
        speed_cruise_ms=drone.speed_cruise_ms,
        speed_max_ms=drone.speed_max_ms,
        speed_vertical_ms=drone.speed_vertical_ms,
        turn_duration_s=drone.turn_duration_s,
        turn_power_factor=drone.turn_power_factor,
        spray_flow_rate_lpm=drone.spray_flow_rate_lpm,
        spray_swath_min_m=drone.spray_swath_min_m,
        spray_swath_max_m=drone.spray_swath_max_m,
        spray_height_m=drone.spray_height_m,
        spray_pump_power_w=drone.spray_pump_power_w,
        app_rate_default_l_ha=drone.app_rate_default_l_ha,
        app_rate_min_l_ha=drone.app_rate_min_l_ha,
        app_rate_max_l_ha=drone.app_rate_max_l_ha,
        service_time_s=drone.service_time_s,
    )


@router.get("/", response_model=list[DroneSummary])
def list_drones(db: Session = Depends(get_db)):
    drones = db.query(Drone).order_by(Drone.name).all()
    return [_serialize_drone_summary(drone) for drone in drones]


@router.get("/{name}/defaults", response_model=DroneDefaults)
def get_drone_defaults(name: str, db: Session = Depends(get_db)):
    return _serialize_drone_defaults(_drone_or_404(name, db))


@router.get("/{name}", response_model=DroneDetail)
def get_drone(name: str, db: Session = Depends(get_db)):
    return _serialize_drone_detail(_drone_or_404(name, db))
