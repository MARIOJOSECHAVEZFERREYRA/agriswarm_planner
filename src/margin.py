import numpy as np
from shapely.geometry import Polygon

class MarginReducer:
    """
    Implementación de la Fase 1: Reducción de Márgenes (Boundary Shrinking).
    Basado en las Ecuaciones 1, 2 y 3 del paper de Li et al. (2023).
    """

    @staticmethod
    def shrink(polygon: Polygon, margin_h: float) -> Polygon:
        """
        Contrae un polígono una distancia 'h' hacia su interior.
        
        :param polygon: Polígono shapely original.
        :param margin_h: Distancia de seguridad en metros (h).
        :return: Nuevo Polígono reducido.
        """
        # 1. Asegurar orientación Anti-Horaria (CCW) para consistencia matemática
        # En Shapely/GIS, CCW significa que el interior está a la izquierda.
        if polygon.exterior.is_ccw:
            coords = np.array(polygon.exterior.coords)[:-1] # Quitamos el último punto repetido
        else:
            coords = np.array(polygon.exterior.coords)[::-1][:-1]

        num_points = len(coords)
        new_coords = []

        for i in range(num_points):
            # Obtener vértices: Anterior (prev), Actual (curr), Siguiente (next)
            prev_p = coords[i - 1]
            curr_p = coords[i]
            next_p = coords[(i + 1) % num_points]

            # --- ECUACIÓN 1: Cálculo del Vector Bisectriz ---
            # Vectores que salen del vértice actual hacia los vecinos
            vec_prev = prev_p - curr_p
            vec_next = next_p - curr_p

            # Normalizar vectores (hacerlos unitarios)
            len_prev = np.linalg.norm(vec_prev)
            len_next = np.linalg.norm(vec_next)
            
            # Evitar división por cero si hay puntos duplicados
            if len_prev == 0 or len_next == 0:
                new_coords.append(curr_p)
                continue

            u_prev = vec_prev / len_prev
            u_next = vec_next / len_next

            # Vector Suma (Dirección de la bisectriz)
            # Nota: Apunta hacia el "interior" del ángulo formado por las líneas
            vec_C = u_prev + u_next
            
            # --- ECUACIÓN 2 y Detección de Concavidad ---
            # Calculamos el ángulo interior theta usando producto punto
            # Dot product: a . b = |a||b| cos(theta)
            dot_prod = np.dot(u_prev, u_next)
            # Clampear valor para evitar errores numéricos en arccos (ej: 1.00000001)
            dot_prod = np.clip(dot_prod, -1.0, 1.0)
            theta = np.arccos(dot_prod)

            # Detectar si el ángulo es Cóncavo (Reflex) usando Producto Cruz (2D)
            # Cross product (z-component): a_x*b_y - a_y*b_x
            cross_prod = np.cross(u_next, u_prev)
            
            # Determinar si es convexo o cóncavo
            # En recorrido CCW, si cross > 0 el giro es a la izquierda (convexo estándar)
            # Si cross < 0, es un giro a la derecha ("mordida" hacia adentro -> cóncavo)
            is_convex = cross_prod > 0

            # --- ECUACIÓN 3: Magnitud del Desplazamiento ---
            # Distancia a mover el vértice: L = h / sin(theta/2)
            # Nota: theta calculado por arccos es siempre [0, pi], que es el ángulo interno 
            # (o externo dependiendo de la referencia), pero para la magnitud sirve.
            # Sin embargo, si los vectores son colineales (theta=180), sin(90)=1 -> L=h (correcto)
            # Si theta=0 (aguja), sin(0)=0 -> L=infinito (correcto, no se puede reducir una aguja)
            
            sin_half_theta = np.sin(theta / 2.0)
            
            if sin_half_theta < 1e-6:
                # Caso degenerado (línea recta o aguja muy afilada), usamos h directo o saltamos
                offset_magnitude = margin_h
            else:
                offset_magnitude = margin_h / sin_half_theta

            # --- Dirección Final del Desplazamiento ---
            norm_C = np.linalg.norm(vec_C)
            if norm_C < 1e-6:
                # Caso especial: vectores opuestos (180 grados). La bisectriz es perpendicular.
                # Rotamos u_next 90 grados a la izquierda (CCW)
                dir_vector = np.array([-u_next[1], u_next[0]])
            else:
                dir_vector = vec_C / norm_C

            # APLICAR EL PAPER :
            # "Concave point has the opposite shift direction"
            # Si es convexo, vec_C apunta hacia adentro del polígono.
            # Si es cóncavo, vec_C apunta hacia afuera ("boca de pacman"), 
            # pero queremos reducir el polígono, así que debemos mover el borde "hacia la carne".
            # En un punto cóncavo, movernos hacia adentro significa ir CONTRA vec_C.
            
            if is_convex:
                final_movement = dir_vector * offset_magnitude
            else:
                # Invertimos dirección para puntos cóncavos
                # Nota: Dependiendo de la geometría exacta, a veces el vector suma ya apunta
                # hacia afuera. Verificamos visualmente:
                # Convexo (V): Suma apunta dentro. Queremos ir dentro. OK.
                # Cóncavo (L interna): Suma apunta al vacío. Queremos ir a la carne (dentro).
                # Por tanto, necesitamos invertir.
                final_movement = -dir_vector * offset_magnitude

            new_p = curr_p + final_movement
            new_coords.append(new_p)

        # Cerrar el polígono y devolver
        if len(new_coords) < 3:
            return polygon # Fallo, devuelve original o vacío
            
        return Polygon(new_coords)