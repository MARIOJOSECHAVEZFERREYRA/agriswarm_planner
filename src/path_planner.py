# src/path_planner.py
import numpy as np
from shapely.geometry import Polygon, LineString, MultiLineString
from shapely import affinity
from typing import List, Tuple

class CoveragePlanner:
    """
    Genera rutas de cobertura (Coverage Path Planning) dentro de un polígono.
    Incluye optimización de ángulo de barrido.
    """

    def __init__(self, sweep_width: float = 5.0):
        self.sweep_width = sweep_width

    def generate_optimized_path(self, polygon: Polygon) -> Tuple[List[list], float]:
        """
        Prueba varios ángulos y devuelve la ruta con menos giros.
        :return: (lista_de_coordenadas, angulo_optimo)
        """
        best_path = []
        min_turns = float('inf')
        best_angle = 0
        
        # Probar rotaciones cada 15 grados
        for angle in range(0, 180, 15):
            path = self._generate_path_at_angle(polygon, angle)
            turns = len(path) # Cada línea es un tramo, más tramos = más giros
            
            if turns < min_turns and turns > 0:
                min_turns = turns
                best_path = path
                best_angle = angle
                
        return best_path, best_angle

    def _generate_path_at_angle(self, polygon: Polygon, angle: float) -> List[list]:
        # 1. Guardar el centroide FIJO del polígono original
        centroid = polygon.centroid
        
        # 2. Rotar el polígono usando ese centroide
        rotated_poly = affinity.rotate(polygon, -angle, origin=centroid)
        min_x, min_y, max_x, max_y = rotated_poly.bounds
        
        lines = []
        y_current = min_y + (self.sweep_width / 2)
        direction = True 
        
        while y_current < max_y:
            sweepline = LineString([(min_x - 1000, y_current), (max_x + 1000, y_current)])
            intersection = sweepline.intersection(rotated_poly)
            
            if not intersection.is_empty:
                if isinstance(intersection, MultiLineString):
                    segs = list(intersection.geoms)
                else:
                    segs = [intersection]
                
                segs.sort(key=lambda s: s.coords[0][0])

                for seg in segs:
                    coords = list(seg.coords)
                    if not direction:
                        coords.reverse()
                    
                    # 3. AQUÍ ESTÁ EL CAMBIO:
                    # Creamos la línea temporalmente
                    line_tmp = LineString(coords)
                    
                    # Des-rotamos usando EL MISMO CENTROIDE del polígono (no 'centroid')
                    restored_line = affinity.rotate(line_tmp, angle, origin=centroid)
                    
                    # Guardamos las coordenadas finales
                    lines.append(list(restored_line.coords))
            
            y_current += self.sweep_width
            direction = not direction 
            
        return lines