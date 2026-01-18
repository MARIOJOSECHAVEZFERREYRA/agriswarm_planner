from shapely.geometry import Point, LineString
import numpy as np
from .mobile_station import MobileStation

class MissionSegmenter:
    """
    Corta una ruta continua en segmentos operables basandose en la fisica
    del dron (Bateria, Liquido) y la logistica del camion.
    """
    
    def __init__(self, drone_specs, mobile_station, target_rate_l_ha=20.0, work_speed_kmh=20.0):
        self.specs = drone_specs
        self.station = mobile_station
        self.rate_l_ha = target_rate_l_ha
        self.speed_kmh = float(drone_specs.flight.work_speed_kmh.value) # Override with spec or param? 
        # Usually specs define MAX, but mission defines OPERATING. Lets use param or default to spec.
        if work_speed_kmh:
             self.speed_kmh = work_speed_kmh
        
        self.speed_ms = self.speed_kmh / 3.6
        self.swath_width = 5.0 # Default fallback
        if self.specs.spray and self.specs.spray.swath_m:
             self.swath_width = (float(self.specs.spray.swath_m[0].value) + float(self.specs.spray.swath_m[1].value)) / 2
             
        # Physics Constants
        self.liters_per_meter = (self.rate_l_ha * self.swath_width) / 10000.0
        
        # Tank Capacity
        self.tank_capacity = 0.0
        if self.specs.spray and self.specs.spray.tank_l:
            self.tank_capacity = float(self.specs.spray.tank_l.value)

        # Battery / Endurance (Simplified to Time for now, could be Energy)
        # Using Hover Loaded as worst case conservative estimate for Work Time
        # Real work consumption is usually between Hover Empty and Hover Loaded.
        self.max_endurance_min = 15.0 # Default
        if self.specs.flight and self.specs.flight.flight_time_min:
             self.max_endurance_min = float(self.specs.flight.flight_time_min['hover_loaded'].value)
        self.max_endurance_s = self.max_endurance_min * 60.0

    def validate_pump(self):
        """
        Verifica si la bomba del dron soporta el caudal demandado.
        :return: (bool, message, required_flow)
        """
        if not self.specs.spray or not self.specs.spray.max_flow_l_min:
            return True, "No pump specs", 0
            
        req_flow_l_min = (self.rate_l_ha * self.speed_kmh * self.swath_width) / 600.0
        max_flow = float(self.specs.spray.max_flow_l_min.value)
        
        if req_flow_l_min > max_flow:
            return False, f"Pump overload! Req: {req_flow_l_min:.1f} L/min > Max: {max_flow}", req_flow_l_min
            
        return True, "OK", req_flow_l_min

    def segment_path(self, polygon, raw_path, truck_polygon=None):
        """
        Segmenta la ruta.
        :param polygon: Shapely Polygon del campo (cultivo).
        :param raw_path: Lista de tuplas [(x,y,z), ...]
        :param truck_polygon: Shapely Polygon de la ruta del camion (buffer exterior). Si se provee, los R_opt estaran aqui.
        :return: List[dict] -> Ciclos de mision
        """
        segments = []
        current_segment = []
        
        # Poligono de referencia para el camion (Donde vive el camion)
        # Si no nos dan uno especifico, asumimos que el camion va por el borde del cultivo (comportamiento original)
        ref_polygon_truck = truck_polygon if truck_polygon else polygon
        
        # Estado actual del dron
        current_liquid = self.tank_capacity
        current_time_air = 0.0
        
        # 0. INICIALIZACION CORRECTA DEL CAMION
        # El camion NO empieza dentro del campo. Empieza en el perimetro EXTERIOR (ref_polygon_truck),
        # en el punto mas cercano al primer punto de trabajo del dron.
        p_start = raw_path[0]
        r_start, _, _ = self.station.calculate_rendezvous(ref_polygon_truck, p_start[:2], p_start[:2]) 
        truck_pos = (r_start.x, r_start.y)
        
        # Safety Thresholds
        safe_time_return = 120.0 # 2 mins reserve for landing/RTH
        
        for i in range(len(raw_path) - 1):
            p1 = raw_path[i]
            p2 = raw_path[i+1]
            
            # 1. Calcular costo de este tramo (P1 -> P2)
            dist_step = np.linalg.norm(np.array(p1[:2]) - np.array(p2[:2]))
            time_step = dist_step / self.speed_ms
            liq_step = dist_step * self.liters_per_meter
            
            # 2. Predecir costo de RETORNO desde P2 (el final de este paso)
            # Calculamos R_opt desde P2 sobre el camino del camion
            r_opt_p2, dist_truck, time_truck = self.station.calculate_rendezvous(ref_polygon_truck, p2[:2], truck_pos[:2])
            dist_return_air = np.linalg.norm(np.array(p2[:2]) - np.array([r_opt_p2.x, r_opt_p2.y]))
            time_return_air = dist_return_air / (self.speed_ms * 1.5) # Vuelve mas rapido (vacío)
            
            total_time_if_commit = current_time_air + time_step + time_return_air + safe_time_return
            liquid_if_commit = current_liquid - liq_step
            
            # 3. Decision Logic: Podemos hacer este paso Y volver?
            CAN_DO_STEP = (total_time_if_commit <= self.max_endurance_s) and (liquid_if_commit >= 0)
            
            if CAN_DO_STEP:
                # Ejecutar paso
                current_segment.append(p1)
                if i == len(raw_path) - 2: # Ultimo punto
                    current_segment.append(p2)
                    
                current_liquid -= liq_step
                current_time_air += time_step
            else:
                # NO PODEMOS. Cortar en P1
                current_segment.append(p1) 
                
                # Calcular R_opt real desde P1
                r_opt_p1, dist_truck_p1, time_truck_p1 = self.station.calculate_rendezvous(ref_polygon_truck, p1[:2], truck_pos[:2])
                r_point = (r_opt_p1.x, r_opt_p1.y)
                
                # --- LOGICA DE VIAJE (COMMUTE) ---
                full_flight_path = [truck_pos] + current_segment + [r_point]
                
                # Guardar ciclo
                segments.append({
                    "type": "work",
                    "path": full_flight_path, 
                    "truck_start": truck_pos,
                    "truck_end": r_point,
                    "truck_dist": dist_truck_p1
                })
                
                # RESET STATE
                # Iniciamos el siguiente ciclo EN p1 para no perder el tramo p1->p2
                current_segment = [p1] 
                truck_pos = r_point 
                
                # Asumimos bateria/tanque llenos
                current_liquid = self.tank_capacity
                current_time_air = 0.0
                
                # IMPORTANTE: Ya que hemos decidido hacer el tramo p1->p2 en el NUEVO ciclo,
                # debemos descontar su costo ahora mismo.
                current_liquid -= liq_step
                current_time_air += time_step
                
                # Opcional: Verificar si incluso con tanque lleno no podemos hacer este paso
                # (caso de linea inmensamente larga). Por ahora asumimos que cabe.

        # Agregar el ultimo segmento remanente
        if current_segment:
             # Calcular punto optimo de recogida final (R_end)
             p_last = current_segment[-1]
             
             r_end, dist_truck_final, time_truck_final = self.station.calculate_rendezvous(ref_polygon_truck, p_last[:2], truck_pos[:2])
             r_end_point = (r_end.x, r_end.y)
             
             # Agregar trayectos de despegue (desde ult truck pos) y aterrizaje final
             full_flight_path = [truck_pos] + current_segment + [r_end_point]

             segments.append({
                    "type": "work",
                    "path": full_flight_path,
                    "truck_start": truck_pos,
                    "truck_end": r_end_point, 
                    "truck_dist": dist_truck_final
             })
             
        return segments
