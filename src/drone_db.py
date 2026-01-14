from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List, Tuple

# ----------------------------
# DEFINICIONES DE ESTRUCTURA
# ----------------------------

@dataclass(frozen=True)
class SpecValue:
    """Contenedor de valor con trazabilidad."""
    value: Any
    units: str
    conditions: str = ""
    source: str = ""

@dataclass(frozen=True)
class FlightSpec:
    max_speed_kmh: SpecValue
    work_speed_kmh: Optional[SpecValue] = None
    max_wind_mps: Optional[SpecValue] = None
    flight_time_min: Dict[str, SpecValue] = field(default_factory=dict)
    flight_distance_km: Dict[str, SpecValue] = field(default_factory=dict)

@dataclass(frozen=True)
class PhysicalSpec:
    width_m: Optional[SpecValue] = None 
    length_m: Optional[SpecValue] = None
    height_m: Optional[SpecValue] = None
    weight_empty_kg: Optional[SpecValue] = None
    weight_max_takeoff_kg: Optional[SpecValue] = None

@dataclass(frozen=True)
class BatterySpec:
    model: str
    energy_wh: Optional[SpecValue] = None
    voltage_v: Optional[SpecValue] = None
    capacity_ah: Optional[SpecValue] = None
    charge_time_min: Optional[SpecValue] = None
    hot_swap: Optional[SpecValue] = None

@dataclass(frozen=True)
class SpraySpec:
    tank_l: SpecValue
    swath_m: Optional[Tuple[SpecValue, SpecValue]] = None 
    max_flow_l_min: Optional[SpecValue] = None
    application_rate_l_ha: Optional[Tuple[SpecValue, SpecValue]] = None
    nozzle_count: Optional[SpecValue] = None
    droplet_vmd_um: Optional[Tuple[SpecValue, SpecValue]] = None

@dataclass(frozen=True)
class DroneSpec:
    name: str
    category: str 
    flight: FlightSpec
    physical: Optional[PhysicalSpec] = None
    battery: Optional[BatterySpec] = None
    spray: Optional[SpraySpec] = None
    features: Dict[str, SpecValue] = field(default_factory=dict)

# ----------------------------
# LÓGICA DE BASE DE DATOS (AQUÍ ESTÁ EL CAMBIO NUEVO)
# ----------------------------

class DroneDB:
    DRONES: Dict[str, DroneSpec] = {}

    @staticmethod
    def get_drone_names() -> List[str]:
        return list(DroneDB.DRONES.keys())

    @staticmethod
    def get_specs(drone_name: str) -> Optional[DroneSpec]:
        return DroneDB.DRONES.get(drone_name)

    @staticmethod
    def theoretical_range_km(drone: DroneSpec, time_key: str = "standard", use_work_speed: bool = False) -> Optional[float]:
        time_sv = drone.flight.flight_time_min.get(time_key)
        if not time_sv and drone.flight.flight_time_min:
            time_sv = list(drone.flight.flight_time_min.values())[0]
        if time_sv is None: return None

        speed_sv = drone.flight.work_speed_kmh if use_work_speed and drone.flight.work_speed_kmh else drone.flight.max_speed_kmh
        if speed_sv is None: return None

        try:
            t_h = float(time_sv.value) / 60.0
            v = float(speed_sv.value)
            return v * t_h
        except (TypeError, ValueError):
            return None

    @staticmethod
    def calculate_safety_margin_m(drone: DroneSpec, buffer_gps: float = 0.5) -> float:
        """
        Calcula el margen de seguridad 'h' basado en:
        h = MAX(Radio Físico, Radio Aspersión) + Buffer GPS
        """
        # 1. Radio Físico (Hardware)
        radio_fisico = 0.5 # Default seguro mínimo
        if drone.physical and drone.physical.width_m:
            radio_fisico = float(drone.physical.width_m.value) / 2.0
            
        # 2. Radio Químico (Aspersión)
        radio_spray = 0.0
        if drone.spray and drone.spray.swath_m:
            min_s = float(drone.spray.swath_m[0].value)
            max_s = float(drone.spray.swath_m[1].value)
            avg_swath = (min_s + max_s) / 2.0
            radio_spray = avg_swath / 2.0
            
        # 3. La lógica crítica: El mayor de los dos
        margin = max(radio_fisico, radio_spray) + buffer_gps
        return round(margin, 2)