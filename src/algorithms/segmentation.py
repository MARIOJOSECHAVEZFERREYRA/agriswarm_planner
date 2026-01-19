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

    def _is_spraying(self, p1, p2, polygon):
        """
        Determina si un segmento es de fumigacion (Grid Line dentro del campo)
        o de transito (Deadheading/Turn).
        """
        line = LineString([p1[:2], p2[:2]])
        mid = line.interpolate(0.5, normalized=True)
        
        # Strict check for Inside
        is_inside = polygon.buffer(1e-9).contains(mid)
        if not is_inside:
            return False
            
        # Heuristic: Turns usually run along the boundary and are short.
        # We want to mark them as Transit (False) to avoid ugly "block" swaths on turns.
        length = np.linalg.norm(np.array(p1[:2]) - np.array(p2[:2]))
        dist_bound = polygon.boundary.distance(mid)
        
        # If segment is on/near boundary and short (relative to swath), assume it's a turn.
        if dist_bound < 1.0 and length < self.swath_width * 2.5:
            # print(f"DEBUG: Turn detected at {mid}, L={length:.1f}, DistB={dist_bound:.2f} -> Transit")
            return False
            
        return True

    def segment_path(self, polygon, raw_path, truck_polygon=None):
        """
        Segmenta la ruta con logica Smart Nozzle.
        """
        cycles = []
        current_cycle_points = []
        current_cycle_segments = [] # List of {'p1':, 'p2':, 'spraying': bool}
        
        ref_polygon_truck = truck_polygon if truck_polygon else polygon
        
        # Estado actual
        current_liquid = self.tank_capacity
        current_time_air = 0.0
        
        # Initial Truck Pos
        p_start = raw_path[0]
        r_start, _, _, _ = self.station.calculate_rendezvous(ref_polygon_truck, p_start[:2], p_start[:2]) 
        truck_pos = (r_start.x, r_start.y)
        
        # Start Cycle Logic
        # Commute 1: Truck -> First Point (DEADHEADING)
        # We handle commutes DYNAMICALLY when flushing a cycle or starting a new one.
        # But for the loop, we need to account for the "Initial Commute" cost in the FIRST cycle.
        
        # Init first cycle
        # We need to calculate initial commute p1->p2 cost? No, Truck->P1 cost.
        dist_commute_in = np.linalg.norm(np.array(truck_pos) - np.array(p_start[:2]))
        time_commute_in = dist_commute_in / self.speed_ms
        current_time_air += time_commute_in
        # No liquid for commute
        
        last_point_added = truck_pos
        
        i = 0
        while i < len(raw_path) - 1:
            p1 = raw_path[i]
            p2 = raw_path[i+1]
            
            # 1. Analyze Segment (Spray vs Deadhead)
            is_spray = self._is_spraying(p1, p2, polygon)
            
            dist_step = np.linalg.norm(np.array(p1[:2]) - np.array(p2[:2]))
            time_step = dist_step / self.speed_ms
            
            # Liquid only if Spraying
            liq_step = (dist_step * self.liters_per_meter) if is_spray else 0.0
            
            # 2. Predict Return Cost from P2
            r_opt_p2, _, _, _ = self.station.calculate_rendezvous(ref_polygon_truck, p2[:2], truck_pos[:2])
            dist_return = np.linalg.norm(np.array(p2[:2]) - np.array([r_opt_p2.x, r_opt_p2.y]))
            time_return = dist_return / (self.speed_ms * 1.5)
            
            full_cycle_time = current_time_air + time_step + time_return + 120.0 # Safety
            full_cycle_liq = current_liquid - liq_step # Return no gasta liq
            
            CAN_DO = (full_cycle_time <= self.max_endurance_s) and (full_cycle_liq >= 0)
            
            if CAN_DO:
                # Add segment
                seg = {'p1': p1, 'p2': p2, 'spraying': is_spray}
                current_cycle_segments.append(seg)
                current_cycle_points.append(p1) # Only store starts, append end later
                
                current_liquid -= liq_step
                current_time_air += time_step
                last_point_added = p2
                i += 1
            else:
                # CUT CYCLE at P1
                # Finalize current cycle
                current_cycle_points.append(p1) # Close loop at P1
                
                # Calculate return stats
                r_opt_p1, dist_truck, _, truck_path_list = self.station.calculate_rendezvous(ref_polygon_truck, p1[:2], truck_pos[:2])
                r_point = (r_opt_p1.x, r_opt_p1.y)
                
                # Add Return Segment (DEADHEADING)
                commute_return = {'p1': p1, 'p2': r_point, 'spraying': False}
                current_cycle_segments.append(commute_return)
                
                # Add Initial Commute (Prepend) - wait, we tracked cost but didn't store segment object?
                # We need to reconstruct the FULL list of segments for the GUI.
                # The "current_cycle_segments" currently starts from P_start.
                # We need to prepend [Truck -> P_cycle_start].
                
                # Find P_cycle_start
                if current_cycle_segments:
                    p_cyc_start = current_cycle_segments[0]['p1']
                    commute_in = {'p1': truck_pos, 'p2': p_cyc_start, 'spraying': False}
                    current_cycle_segments.insert(0, commute_in)
                    
                    # Full Path Points
                    full_path = [truck_pos] + current_cycle_points + [r_point]
                else:
                    # Edge case: Can't even do first segment?
                    full_path = []

                # Save Cycle
                cycles.append({
                    "type": "work",
                    "path": full_path,
                    "segments": current_cycle_segments, # NEW METADATA
                    "truck_start": truck_pos,
                    "truck_end": r_point,
                    "truck_start": truck_pos,
                    "truck_end": r_point,
                    "truck_dist": dist_truck,
                    "truck_path_coords": truck_path_list # Store Geometry
                })
                
                # RESET & SETUP NEW CYCLE
                truck_pos = r_point
                current_liquid = self.tank_capacity
                current_time_air = 0.0
                current_cycle_segments = []
                current_cycle_points = []
                
                # Costo de entrada al nuevo ciclo: Truck -> P1 (donde nos quedamos)
                dist_commute_in = np.linalg.norm(np.array(truck_pos) - np.array(p1[:2]))
                time_commute_in = dist_commute_in / self.speed_ms
                current_time_air += time_commute_in
                
                # NO incrementamos i, reintentamos P1->P2 en el nuevo ciclo
                
        # Final Cycle
        if current_cycle_segments:
             p_last = current_cycle_segments[-1]['p2']
             r_end, dist_truck_final, _, truck_path_list_final = self.station.calculate_rendezvous(ref_polygon_truck, p_last[:2], truck_pos[:2])
             r_end_point = (r_end.x, r_end.y)
             
             # Return Segment
             commute_return = {'p1': p_last, 'p2': r_end_point, 'spraying': False}
             current_cycle_segments.append(commute_return)
             
             # Prepend Start Commute
             p_cyc_start = current_cycle_segments[0]['p1']
             commute_in = {'p1': truck_pos, 'p2': p_cyc_start, 'spraying': False}
             current_cycle_segments.insert(0, commute_in)
             
             # Points
             current_cycle_points.append(p_last)
             full_path = [truck_pos] + current_cycle_points + [r_end_point]
             
             cycles.append({
                    "type": "work",
                    "path": full_path,
                    "segments": current_cycle_segments,
                    "truck_start": truck_pos,
                    "truck_end": r_end_point, 
                    "truck_end": r_end_point, 
                    "truck_dist": dist_truck_final,
                    "truck_path_coords": truck_path_list_final
             })
             
        return cycles
