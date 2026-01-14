import json
import os
from shapely.geometry import Polygon

class FieldIO:
    """
    Módulo para guardar y cargar campos agrícolas en formato JSON simple.
    """

    @staticmethod
    def save_field(polygon: Polygon, filename: str):
        """Guarda las coordenadas del polígono en un archivo JSON."""
        # Asegurar que el directorio existe
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        # Extraer coordenadas como lista de tuplas
        coords = list(polygon.exterior.coords)
        
        data = {
            "type": "Polygon",
            "coordinates": coords
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"💾 Campo guardado en: {filename}")

    @staticmethod
    def load_field(filename: str) -> Polygon:
        """Carga un polígono desde un archivo JSON."""
        if not os.path.exists(filename):
            raise FileNotFoundError(f"No se encontró el archivo: {filename}")
            
        with open(filename, 'r') as f:
            data = json.load(f)
            
        return Polygon(data["coordinates"])