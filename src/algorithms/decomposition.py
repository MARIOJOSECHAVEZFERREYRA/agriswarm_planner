import numpy as np
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import split
import math

class ConcaveDecomposer:
    """
    Implementación de la Fase 2: Detección de Concavidad y Descomposición.
    Basado en las Secciones 2.3 y 2.4 del paper de Li et al. (2023).
    """

    @staticmethod
    def decompose(polygon: Polygon, heading_angle_deg: float, depth: int = 0):
        """
        Función recursiva principal.
        Verifica si el polígono tiene concavidades 'Tipo 2' que obstruyan el vuelo
        en el ángulo dado. Si las hay, corta el polígono y procesa las partes.
        
        :return: Lista de Polígonos convexos (o seguros para volar).
        """
        if depth > 50:
            print("Max Recursion Depth Reached. Returning original polygon.")
            return [polygon]

        # Convertir ángulo a radianes para cálculos trigonométricos
        heading_rad = np.radians(heading_angle_deg)
        
        # 1. Obtener coordenadas
        coords = list(polygon.exterior.coords)
        if coords[0] == coords[-1]:
            coords = coords[:-1]
        n = len(coords)
        
        # 2. Buscar el PRIMER punto cóncavo que sea "Tipo 2" (obstructivo)
        for i in range(n):
            if ConcaveDecomposer._is_concave_topology_mapping(coords, i):
                # Si es cóncavo, verificamos si es "Tipo 2" para este ángulo de vuelo
                if ConcaveDecomposer._is_type_2(coords, i, heading_rad):
                    # --- FASE DE CORTE (Section 2.4) ---
                    # Lanzar rayo paralelo al heading y cortar
                    
                    # Debug loop
                    # print(f"[D{depth}] Cutting at vertex {i} {coords[i]} heading {heading_angle_deg}")

                    sub_polygons = ConcaveDecomposer._split_polygon_at_vertex(polygon, coords[i], heading_rad)
                    
                    # Verificación de seguridad: si no se cortó nada, evitar loop infinito
                    if len(sub_polygons) < 2:
                        # print(f"⚠️ Split failed to produce sub-polygons at depth {depth}. Skipping this vertex.")
                        continue 
                        
                    # Verificación de calidad del corte
                    is_trivial = False
                    for sub in sub_polygons:
                        # Reject if split produces a tiny sliver (< 10 m^2) or fails to reduce area significantly (> 99.9%)
                        if sub.area < 10.0 or sub.area > 0.999 * polygon.area:
                            is_trivial = True
                            break
                    
                    if is_trivial:
                        continue # Intentar otro vértice
                        
                    # Recurse on valid split
                    result = []
                    for sub in sub_polygons:
                        result.extend(ConcaveDecomposer.decompose(sub, heading_angle_deg, depth + 1))
                    return result

        # Si no se encontró ninguna concavidad obstructiva, el polígono está listo
        return [polygon]

    @staticmethod
    def _is_concave_topology_mapping(coords, i):
        """
        Detecta si el vértice i es cóncavo usando 'Topology Mapping' (Ec. 8-10).
        """
        n = len(coords)
        curr_p = np.array(coords[i])
        prev_p = np.array(coords[(i - 1) % n])
        next_p = np.array(coords[(i + 1) % n])

        # Paper Section 2.3: Líneas proyectivas L1 y L2
        # El paper define proyecciones basadas en pendiente. 
        # Simplificación robusta equivalente al paper: Producto Cruz.
        # El paper usa mapeo topológico para demostrar matemáticamente lo que hace el producto cruz.
        # Implementamos la lógica vectorial que es computacionalmente estable.
        
        vec_prev = prev_p - curr_p
        vec_next = next_p - curr_p
        
        # Cross product 2D: (x1*y2 - x2*y1)
        cross_prod = vec_next[0] * vec_prev[1] - vec_next[1] * vec_prev[0]
        
        # En Shapely/GIS (orden CCW), un cross negativo indica un giro a la derecha (concavidad)
        # NOTA: Asumimos que el polígono está ordenado CCW (Counter-Clockwise).
        return cross_prod < -1e-3  # Tolerancia aumentada para evitar ruido en vertices casi colineales

    @staticmethod
    def _is_type_2(coords, i, heading_rad):
        """
        Determina si una concavidad es "Tipo 2" (Obstructiva) según Fig. 5 del paper.
        """
        # Vectores del vértice hacia los vecinos
        n = len(coords)
        curr_p = np.array(coords[i])
        prev_p = np.array(coords[(i - 1) % n])
        next_p = np.array(coords[(i + 1) % n])
        
        vec_prev = prev_p - curr_p
        vec_next = next_p - curr_p
        
        # Vector de dirección de vuelo
        flight_vec = np.array([np.cos(heading_rad), np.sin(heading_rad)])
        
        # Para ser Tipo 2 (obstructivo), la línea de vuelo debe entrar "dentro" del polígono
        # en el vértice cóncavo.
        # Geométricamente: El vector de vuelo debe estar ENTRE vec_prev y vec_next
        # dentro del ángulo reflex (el ángulo grande > 180).
        
        # Calculamos ángulos absolutos
        ang_prev = np.arctan2(vec_prev[1], vec_prev[0])
        ang_next = np.arctan2(vec_next[1], vec_next[0])
        ang_flight = np.arctan2(flight_vec[1], flight_vec[0])
        
        # Normalizar a [0, 2pi]
        ang_prev = ang_prev % (2 * np.pi)
        ang_next = ang_next % (2 * np.pi)
        ang_flight = ang_flight % (2 * np.pi)
        
        # Verificar si el vuelo cae en el 'cono' de la concavidad
        # En un punto cóncavo CCW, el ángulo interior es > 180.
        # Si el vuelo pasa por ese ángulo, corta el polígono -> Tipo 2.
        
        if ang_next < ang_prev:
            ang_next += 2 * np.pi
            
        if ang_prev <= ang_flight <= ang_next:
            return True
        if ang_prev <= (ang_flight + 2*np.pi) <= ang_next:
            return True
            
        return False

    @staticmethod
    def _split_polygon_at_vertex(polygon: Polygon, vertex_coords, heading_rad):
        """
        Corta el polígono lanzando un rayo desde el vértice en la dirección del heading.
        """
        # Crear una línea muy larga en la dirección del vuelo
        ray_len = 10000.0 # Longitud arbitraria grande
        ray_end_x = vertex_coords[0] + ray_len * np.cos(heading_rad)
        ray_end_y = vertex_coords[1] + ray_len * np.sin(heading_rad)
        
        cut_line = LineString([vertex_coords, (ray_end_x, ray_end_y)])
        
        # Usar Shapely para dividir
        # Nota: split() puede devolver más de 2 geometrías si es complejo, 
        # pero para un rayo desde un vértice hacia adentro, generalmente son 2.
        result_collection = split(polygon, cut_line)
        
        polys = []
        for geom in result_collection.geoms:
            if isinstance(geom, Polygon):
                polys.append(geom)
        
        return polys