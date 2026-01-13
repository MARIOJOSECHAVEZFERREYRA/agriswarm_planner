# src/geometry.py
from shapely.geometry import Polygon

class FieldGenerator:
    """
    Clase encargada de generar geometrías de campos agrícolas
    para pruebas de simulación.
    """
    
    @staticmethod
    def create_simple_field():
        # Un cuadrado simple de 100x100m
        coords = [(0, 0), (100, 0), (100, 100), (0, 100), (0, 0)]
        return Polygon(coords)

    @staticmethod
    def create_l_shape_field():
        # Un campo en forma de L (más desafiante)
        coords = [(0,0), (50,0), (50,150), (100,150), (100,200), (0,200)]
        return Polygon(coords)