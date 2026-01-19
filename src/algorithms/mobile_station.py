from shapely.geometry import Polygon, Point, LineString
from shapely.ops import substring
import numpy as np

class MobileStation:
    """
    Implementacion de la Sinergia Dron-Camion.
    Basado en la restriccion geometrica: P_truck en el borde del poligono.
    """

    def __init__(self, truck_speed_mps=5.0, truck_offset_m=0.0):
        self.truck_speed = truck_speed_mps # Velocidad media del camion
        self.truck_offset_m = truck_offset_m # Distancia del ruta al borde

    def calculate_rendezvous(self, polygon: Polygon, p_drone_exit: tuple, truck_start_pos: tuple):
        """
        Calcula el punto de encuentro optimo (R_opt) y la logistica.
        """
        # Determine the truck path boundary (optionally offset from field edge)
        if self.truck_offset_m > 0:
            # Buffer expands polygon (positive offset = outside if polygon is CCW and "buffer" logic holds for standard field)
            # Use mitre join (2) to preserve corner shape roughly
            limit_poly = polygon.buffer(self.truck_offset_m, join_style=2)
            boundary = limit_poly.exterior
        else:
            boundary = polygon.exterior
        
        # 1. Encontrar R_opt (Proyeccion ortogonal sobre el borde)
        point_exit = Point(p_drone_exit)
        dist_projected = boundary.project(point_exit)
        r_opt = boundary.interpolate(dist_projected)
        
        # 2. Calcular ruta del Camion sobre el perimetro
        start_dist = boundary.project(Point(truck_start_pos))
        target_dist = dist_projected 
        total_len = boundary.length
        
        # Path 1: CCW (Adelante en el anillo)
        if start_dist <= target_dist:
            path_ccw_geom = substring(boundary, start_dist, target_dist)
        else:
            # Wrap: start->end + 0->target
            p1 = substring(boundary, start_dist, total_len)
            p2 = substring(boundary, 0, target_dist)
            coords = list(p1.coords) + list(p2.coords)
            path_ccw_geom = LineString(coords)
            
        len_ccw = path_ccw_geom.length
        
        # Path 2: CW (Atras en el anillo) -> Calculamos Target->Start (CCW) y lo invertimos
        if target_dist <= start_dist:
            path_cw_rev = substring(boundary, target_dist, start_dist)
        else:
            p1 = substring(boundary, target_dist, total_len)
            p2 = substring(boundary, 0, start_dist)
            coords = list(p1.coords) + list(p2.coords)
            path_cw_rev = LineString(coords)
            
        len_cw = path_cw_rev.length
        
        # Elegir el mas corto
        if len_ccw <= len_cw:
            truck_travel_dist = len_ccw
            path_final_coords = list(path_ccw_geom.coords)
        else:
            truck_travel_dist = len_cw
            # Invertir coordenadas para ir Start->Target
            path_final_coords = list(path_cw_rev.coords)[::-1]

        # 3. Sincronizacion
        truck_time_s = truck_travel_dist / self.truck_speed if self.truck_speed > 0 else float('inf')
        
        return r_opt, truck_travel_dist, truck_time_s, path_final_coords

    def check_feasibility(self, truck_time_s, drone_endurance_s):
        """
        Verifica si el camion llega antes que el dron se caiga.
        """
        # Margen de seguridad (ej. 1 minuto)
        margin_s = 60.0 
        return truck_time_s < (drone_endurance_s - margin_s)
