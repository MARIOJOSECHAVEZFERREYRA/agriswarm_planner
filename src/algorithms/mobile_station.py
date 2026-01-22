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

    def get_road_boundary(self, polygon: Polygon):
        """Devuelve el anillo de la ruta del camion (borde + offset)"""
        if self.truck_offset_m > 0:
            limit_poly = polygon.buffer(self.truck_offset_m, join_style=2)
            return limit_poly.exterior
        return polygon.exterior

    def calculate_rendezvous(self, polygon: Polygon, p_drone_exit: tuple, truck_start_pos: tuple, ref_route: LineString = None):
        """
        Calcula el punto de encuentro optimo (R_opt) y la logistica.
        Si ref_route es != None, usa esa LineString (camino abierto) en lugar del perimetro (anillo).
        """
        if ref_route:
            # LOGICA DE CADENA ABIERTA (Linear Route)
            boundary = ref_route
            
            # CHECK STATIC MODE
            if self.truck_speed < 0.1:
                 # Truck cannot move. Rendezvous is always at truck_start_pos.
                 r_opt = Point(truck_start_pos)
                 if not boundary.distance(r_opt) < 0.1:
                      # Snap if needed (optional)
                      pass
                      
                 return r_opt, 0.0, 0.0, [truck_start_pos]
            
            point_exit = Point(p_drone_exit)
            
            # 1. R_opt (Proyeccion mas cercana en la linea)
            dist_projected = boundary.project(point_exit)
            r_opt = boundary.interpolate(dist_projected)
            
            # 2. Ruta Camion (Lineal, sin vueltas)
            start_dist = boundary.project(Point(truck_start_pos))
            target_dist = dist_projected
            
            truck_travel_dist = abs(target_dist - start_dist)
            
            # Geometria del camino
            if truck_travel_dist > 0.1:
                # Substring siempre devuelve en orden de la linea base
                params = sorted([start_dist, target_dist])
                path_geom = substring(boundary, params[0], params[1])
                path_final_coords = list(path_geom.coords)
                
                # Invertir si vamos "hacia atras" respecto a la definicion de la linea
                if start_dist > target_dist:
                    path_final_coords = path_final_coords[::-1]
            else:
                path_final_coords = [(r_opt.x, r_opt.y)]
                
            truck_time_s = truck_travel_dist / self.truck_speed if self.truck_speed > 0 else float('inf')
            return r_opt, truck_travel_dist, truck_time_s, path_final_coords

        # LOGICA DE ANILLO CERRADO (Perimetro)
        # Determine the truck path boundary
        boundary = self.get_road_boundary(polygon)
        
        # CHECK STATIC MODE
        if self.truck_speed < 0.1:
                # Truck cannot move. Rendezvous is always at truck_start_pos.
                r_opt = Point(truck_start_pos)
                return r_opt, 0.0, 0.0, [truck_start_pos]
                
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
