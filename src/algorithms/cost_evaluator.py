from shapely.geometry import Polygon, Point, LineString
import numpy as np

class RouteCostEvaluator:
    """
    Evaluador de costos cooperativos (Dron + Camión).
    Desacoplado para uso en GA y validación manual.
    """

    @staticmethod
    def calculate_perimeter_distance(polygon: Polygon, p1: tuple, p2: tuple) -> float:
        """
        Calcula la distancia mas corta viajando por el EXTERIOR del poligono.
        Asume que el camion se mueve por el borde (o un offset del borde).
        """
        ring = polygon.exterior
        
        # Proyectar puntos al anillo (para asegurar que esten en el borde)
        d1 = ring.project(Point(p1))
        d2 = ring.project(Point(p2))
        
        # Distancia lineal a lo largo del anillo
        dist_linear = abs(d1 - d2)
        total_length = ring.length
        
        # La distancia mas corta en un anillo es min(arco, total - arco)
        shortest_dist = min(dist_linear, total_length - dist_linear)
        
        return shortest_dist

    @staticmethod
    def calculate_total_truck_cost(polygon: Polygon, drone_path_segments: list) -> float:
        """
        Calcula el costo total del camion sumando los movimientos necesarios
        para conectar los segmentos de vuelo del dron.
        
        :param drone_path_segments: Lista de listas de puntos [[p_start1, ..., p_end1], [p_start2, ..., p_end2]]
                                    donde cada sublista es una ruta dentro de un sub-poligono.
        :return: Distancia total recorrida por el camion (metros)
        """
        if len(drone_path_segments) < 2:
            return 0.0
            
        total_truck_dist = 0.0
        
        # Iterar entre el final de un segmento y el inicio del siguiente
        for i in range(len(drone_path_segments) - 1):
            segment_current = drone_path_segments[i]
            segment_next = drone_path_segments[i+1]
            
            if not segment_current or not segment_next:
                continue
                
            p_end_current = segment_current[-1]
            p_start_next = segment_next[0]
            
            dist = RouteCostEvaluator.calculate_perimeter_distance(polygon, p_end_current, p_start_next)
            total_truck_dist += dist
            
        return total_truck_dist
