from shapely.geometry import Polygon, Point, LineString
from shapely.ops import substring
import numpy as np

class MobileStation:
    """
    Implementacion de la Sinergia Dron-Camion.
    Basado en la restriccion geometrica: P_truck en el borde del poligono.
    """

    def __init__(self, truck_speed_mps=5.0):
        self.truck_speed = truck_speed_mps # Velocidad media del camion (ej. 18 km/h = 5 m/s)

    def calculate_rendezvous(self, polygon: Polygon, p_drone_exit: tuple, truck_start_pos: tuple):
        """
        Calcula el punto de encuentro optimo (R_opt) y la logistica de sincronizacion.
        
        :param polygon: Poligono del campo (el borde es el camino del camion).
        :param p_drone_exit: (x,y) donde el dron termina su mistion o necesita recarga.
        :param truck_start_pos: (x,y) posicion actual del camion.
        :return: (r_opt_point, truck_path, arrival_time_s, is_reachable)
        """
        boundary = polygon.exterior
        
        # 1. Encontrar R_opt (Proyeccion ortogonal sobre el borde)
        # Esto minimiza la distancia Dron->Camion segun la funcion de costo simple.
        point_exit = Point(p_drone_exit)
        dist_projected = boundary.project(point_exit)
        r_opt = boundary.interpolate(dist_projected)
        
        # 2. Calcular ruta del Camion sobre el perimetro
        # El camion debe ir de truck_start_pos a r_opt sobre el anillo.
        # Un anillo es cerrado, asi que hay dos caminos. Elegimos el mas corto.
        
        start_dist = boundary.project(Point(truck_start_pos))
        target_dist = dist_projected # Ya lo tenemos
        
        # Caso A: Ruta directa
        # Ordenamos las distancias para substring
        d_min, d_max = min(start_dist, target_dist), max(start_dist, target_dist)
        
        path_a = substring(boundary, d_min, d_max)
        len_a = path_a.length
        
        # Caso B: Ruta inversa (cruzando el start/end del anillo)
        len_total = boundary.length
        len_b = len_total - len_a
        
        # Determinar cual es el camino REAL que conecta start->target
        # Substring siempre va en direccion del anillo.
        # Si start < target: substring(start, target) es directo (CCW si anillo estandar)
        # Si start > target: substring(start, target) no es posible directo con d_min/d_max normal,
        # hay que componerlo.
        
        # Simplificacion robusta:
        # El camion va por el camino mas corto.
        if len_a <= len_b:
            truck_travel_dist = len_a
            # Reconstruir geometria exacta no es necesario para el calculo de tiempo,
            # pero si visualizamos, podriamos quererla. Por ahora basta la distancia.
        else:
            truck_travel_dist = len_b

        # 3. Sincronizacion
        truck_time_s = truck_travel_dist / self.truck_speed if self.truck_speed > 0 else float('inf')
        
        return r_opt, truck_travel_dist, truck_time_s

    def check_feasibility(self, truck_time_s, drone_endurance_s):
        """
        Verifica si el camion llega antes que el dron se caiga.
        """
        # Margen de seguridad (ej. 1 minuto)
        margin_s = 60.0 
        return truck_time_s < (drone_endurance_s - margin_s)
