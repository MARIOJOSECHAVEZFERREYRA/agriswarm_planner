import numpy as np
from shapely.geometry import Polygon, LineString, MultiLineString, Point
from shapely import affinity
from typing import List, Tuple

class BoustrophedonPlanner:
    """
    Implementación de la Fase 3: Generación de Trayectoria Boustrophedon (Zig-Zag).
    Adaptado para devolver métricas de fitness según Li et al. (2023).
    """

    def __init__(self, spray_width: float = 5.0):
        """
        :param spray_width: Ancho efectivo de aspersión (d) en metros.
        """
        self.spray_width = spray_width

    def generate_path(self, polygon: Polygon, angle_deg: float) -> Tuple[List[tuple], float, float]:
        """
        Genera una ruta de cobertura para un ángulo dado y calcula métricas.
        
        :param polygon: Polígono de la zona de trabajo (debe ser convexo o una sub-zona).
        :param angle_deg: Ángulo de barrido (heading) en grados.
        :return: (waypoints, flight_distance_l, coverage_area_S_prime)
        """
        # 1. Rotar el polígono para alinear el barrido con el eje X horizontal
        # Usamos el centroide para rotar y luego des-rotar sin perder la posición
        centroid = polygon.centroid
        rotated_poly = affinity.rotate(polygon, -angle_deg, origin=centroid)
        
        min_x, min_y, max_x, max_y = rotated_poly.bounds
        
        # Generar líneas de barrido
        lines = []
        y_current = min_y + (self.spray_width / 2)
        direction = True # True = Izquierda -> Derecha
        
        # Métricas internas (en el sistema rotado)
        total_spray_length = 0.0
        
        while y_current < max_y:
            # Línea de barrido infinita
            sweepline = LineString([(min_x - 1000, y_current), (max_x + 1000, y_current)])
            intersection = sweepline.intersection(rotated_poly)
            
            if not intersection.is_empty:
                # Manejar geometrías complejas (MultiLineString)
                if isinstance(intersection, MultiLineString):
                    segs = list(intersection.geoms)
                else:
                    segs = [intersection]
                
                # Ordenar segmentos por coordenada X (siempre de izq a der primero)
                segs.sort(key=lambda s: s.coords[0][0])

                for seg in segs:
                    coords = list(seg.coords)
                    
                    # Calcular longitud de aspersión (para S')
                    # Según Ec. 13: S' = Sum(longitud * d)
                    seg_len = Point(coords[0]).distance(Point(coords[-1]))
                    total_spray_length += seg_len
                    
                    # Implementar Zig-Zag (invertir dirección si toca)
                    if not direction:
                        coords.reverse()
                    
                    lines.append(coords)
            
            y_current += self.spray_width
            direction = not direction # Cambiar sentido para la siguiente línea

        # 2. Construir la Ruta Continua (Unir segmentos)
        # Esto es vital para calcular 'l' (distancia de vuelo real incluyendo giros)
        continuous_path_rotated = []
        if not lines:
            return [], 0.0, 0.0

        for i in range(len(lines)):
            segment = lines[i]
            # Si no es el primer segmento, añadir conexión desde el anterior
            if i > 0:
                prev_end = lines[i-1][-1]
                curr_start = segment[0]
                # Aquí podríamos añadir lógica de giros suaves (Dubins), 
                # pero por ahora usamos conexión directa (línea recta)
                # El paper asume distancia euclidiana para 'l' (Ec. 11)
            
            continuous_path_rotated.extend(segment)

        # 3. Des-rotar la ruta completa para volver a coordenadas GPS/Reales
        final_waypoints = []
        flight_distance_l = 0.0
        
        # Convertir lista de puntos a LineString para rotarla de golpe (más eficiente)
        if len(continuous_path_rotated) > 1:
            path_line = LineString(continuous_path_rotated)
            restored_path = affinity.rotate(path_line, angle_deg, origin=centroid)
            
            # Extraer coordenadas
            final_waypoints = list(restored_path.coords)
            
            # Calcular 'l' (Distancia de Vuelo Total) - Ec. 11
            # flight_distance_l = restored_path.length 
            # (Shapely calcula la longitud geodésica euclidiana correctamente)
            flight_distance_l = restored_path.length
            
        elif len(continuous_path_rotated) == 1:
            # Caso borde: un solo punto
            p = Point(continuous_path_rotated[0])
            restored_p = affinity.rotate(p, angle_deg, origin=centroid)
            final_waypoints = [restored_p.coords[0]]
            flight_distance_l = 0.0

        # 4. Calcular S' (Coverage Area Estimada) - Ec. 13
        # S' = Longitud total de líneas de aspersión * Ancho de aspersión
        coverage_area_s_prime = total_spray_length * self.spray_width

        return final_waypoints, flight_distance_l, coverage_area_s_prime