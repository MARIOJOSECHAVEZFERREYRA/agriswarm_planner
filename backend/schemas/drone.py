from pydantic import BaseModel


class DroneSummary(BaseModel):
    name: str
    num_rotors: int
    default_swath: float
    speed_cruise_ms: float
    speed_max_ms: float
    spray_flow_rate_lpm: float
    battery_capacity_wh: float
    battery_reserve_pct: float
    service_time_s: float


class DroneDefaults(BaseModel):
    swath_m: float
    spray_swath_min_m: float
    spray_swath_max_m: float
    margin_m: float
    app_rate_l_ha: float
    app_rate_min_l_ha: float
    app_rate_max_l_ha: float
    speed_ms: float
    speed_min_ms: float
    speed_max_ms: float


class DroneDetail(BaseModel):
    id: int
    name: str
    num_rotors: int
    mass_empty_kg: float
    mass_battery_kg: float
    mass_tank_full_kg: float
    mass_takeoff_max_kg: float
    battery_capacity_wh: float
    battery_voltage_v: float
    battery_reserve_pct: float
    battery_charge_time_min: float
    power_hover_empty_w: float
    power_hover_full_w: float
    speed_cruise_ms: float
    speed_max_ms: float
    speed_vertical_ms: float
    turn_duration_s: float
    turn_power_factor: float
    spray_flow_rate_lpm: float
    spray_swath_min_m: float
    spray_swath_max_m: float
    spray_height_m: float
    spray_pump_power_w: float
    app_rate_default_l_ha: float
    app_rate_min_l_ha: float
    app_rate_max_l_ha: float
    service_time_s: float
