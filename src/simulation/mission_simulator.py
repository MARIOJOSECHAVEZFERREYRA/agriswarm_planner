import math
from shapely.geometry import Point, LineString

class MissionSimulator:
    def __init__(self, specs):
        self.specs = specs
        
        # Bateria (con margen de seguridad)
        self.max_flight_time_min = float(specs.flight.flight_time_min['hover_loaded'].value)
        self.safe_flight_time_min = self.max_flight_time_min * 0.8 
        self.speed_kmh = float(specs.flight.work_speed_kmh.value)
        self.speed_ms = self.speed_kmh / 3.6
        self.max_battery_dist_m = self.speed_ms * (self.safe_flight_time_min * 60)

        # Tanque
        self.tank_l = float(specs.spray.tank_l.value)
        swath_min = float(specs.spray.swath_m[0].value)
        swath_max = float(specs.spray.swath_m[1].value)
        self.swath_width = (swath_min + swath_max) / 2.0
        self.application_rate = 20.0 
        
        liters_per_sqm = self.application_rate / 10000.0
        max_area_m2 = self.tank_l / liters_per_sqm
        self.max_tank_dist_m = max_area_m2 / self.swath_width

    def split_mission(self, raw_path, road_ring):
        """Divide la mision buscando siempre el retorno mas corto al camino."""
        cycles = []
        rendezvous_points = []
        
        # 1. Inicio: Punto del camino mas cercano al primer waypoint
        first_wp = Point(raw_path[0])
        dist_on_road = road_ring.project(first_wp)
        home_point = road_ring.interpolate(dist_on_road)
        
        current_pos = (home_point.x, home_point.y)
        current_cycle = [current_pos]
        rendezvous_points.append(current_pos)
        
        cycle_dist_battery = 0.0
        cycle_dist_tank = 0.0
        
        i = 0
        while i < len(raw_path):
            target_point = raw_path[i]
            
            # Distancia al siguiente punto de trabajo
            dist_seg = math.sqrt((target_point[0]-current_pos[0])**2 + (target_point[1]-current_pos[1])**2)
            
            # CALCULO INTELIGENTE:
            # Si voy al target, ¿Cual es la salida mas cercana desde ESE target?
            t_geom = Point(target_point)
            proj_dist = road_ring.project(t_geom)
            nearest_exit_from_target = road_ring.interpolate(proj_dist)
            
            # Distancia de escape hipotetica
            dist_escape = math.sqrt((target_point[0]-nearest_exit_from_target.x)**2 + (target_point[1]-nearest_exit_from_target.y)**2)
            
            # Consumos Futuros (Ida + Escape)
            future_battery = cycle_dist_battery + dist_seg + dist_escape
            future_tank = cycle_dist_tank + dist_seg
            
            # --- DECISION ---
            if future_battery > self.max_battery_dist_m or future_tank > self.max_tank_dist_m:
                # NO ALCANZAMOS. Debemos salir YA desde donde estamos.
                
                # Buscamos la salida mas cercana desde mi posicion ACTUAL (current_pos)
                # Esto evita cruzar el campo diagonalmente. Se sale por la tangente.
                curr_geom = Point(current_pos)
                proj_curr = road_ring.project(curr_geom)
                emergency_exit = road_ring.interpolate(proj_curr)
                exit_coords = (emergency_exit.x, emergency_exit.y)
                
                # Cerrar ciclo
                current_cycle.append(exit_coords)
                cycles.append(current_cycle)
                rendezvous_points.append(exit_coords)
                
                # Iniciar nuevo ciclo (Recarga)
                current_cycle = [exit_coords]
                current_pos = exit_coords
                cycle_dist_battery = 0.0
                cycle_dist_tank = 0.0
                
                # No incrementamos 'i', reintentamos ir al target con bateria llena
            else:
                # Alcanzamos, seguimos
                cycle_dist_battery += dist_seg
                cycle_dist_tank += dist_seg
                current_cycle.append(target_point)
                current_pos = target_point
                i += 1

        # Cerrar el ultimo ciclo
        # Salir por el punto mas cercano al ultimo waypoint
        last_geom = Point(current_pos)
        proj_last = road_ring.project(last_geom)
        final_exit = road_ring.interpolate(proj_last)
        final_coords = (final_exit.x, final_exit.y)
        
        current_cycle.append(final_coords)
        cycles.append(current_cycle)
        rendezvous_points.append(final_coords)
        
        return cycles, rendezvous_points