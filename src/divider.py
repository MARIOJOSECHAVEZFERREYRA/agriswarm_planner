# src/divider.py
from shapely.geometry import Polygon
from typing import List

class FieldDivider:
    """
    Clase responsable de dividir un campo de cultivo en sub-áreas
    para asignar a múltiples drones.
    """

    @staticmethod
    def divide_vertically(field: Polygon, num_drones: int) -> List[Polygon]:
        """
        Divide el campo en franjas verticales simples.
        :param field: Polígono del campo completo.
        :param num_drones: Cantidad de partes a generar.
        :return: Lista de sub-polígonos.
        """
        min_x, min_y, max_x, max_y = field.bounds
        total_width = max_x - min_x
        slice_width = total_width / num_drones
        
        sub_areas = []
        
        for i in range(num_drones):
            # Calcular coordenadas de la "caja de corte"
            left = min_x + (i * slice_width)
            right = min_x + ((i + 1) * slice_width)
            
            # Crear la caja de corte (slice)
            cut_box = Polygon([
                (left, min_y), (right, min_y),
                (right, max_y), (left, max_y)
            ])
            
            # Intersección segura
            chunk = field.intersection(cut_box)
            
            # Solo agregar si el pedazo no está vacío (evita errores con formas raras)
            if not chunk.is_empty:
                sub_areas.append(chunk)
                
        return sub_areas