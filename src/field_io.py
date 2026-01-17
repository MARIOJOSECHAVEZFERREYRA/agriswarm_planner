import json
import os
import numpy as np
from shapely.geometry import Polygon
from typing import List, Optional
from algorithms.map_interface import MapInterface

class JSONMapAdapter(MapInterface):
    """
    Adapter to load legacy JSON maps via the MapInterface.
    """
    def __init__(self, filename: str):
        self.filename = filename
        self._data = self._load()

    def _load(self):
        if not os.path.exists(self.filename):
            raise FileNotFoundError(f"Map file not found: {self.filename}")
        with open(self.filename, 'r') as f:
            return json.load(f)

    def get_boundary(self) -> Polygon:
        # Legacy JSON format had "coordinates" directly or under "features" if GeoJSON.
        # The checked file uses {"coordinates": [...]}
        coords = self._data.get("coordinates", [])
        
        # Ensure 3D support (padded with 0 if missing)
        # However, for now we return 2D polygon as shapely handles 2D better for planning
        # The interface allows future extensions.
        if not coords:
            return Polygon()
        return Polygon(coords)

    def get_obstacles(self) -> List[Polygon]:
        # Legacy format might not have obstacles, return empty list if not found
        obs_data = self._data.get("obstacles", [])
        return [Polygon(obs) for obs in obs_data]

    def get_metadata(self) -> dict:
        return {
            "source": "json",
            "filename": self.filename,
            "type": self._data.get("type", "unknown")
        }

class FieldIO:
    """
    Legacy IO helper, now utilizing or acting as a factory for Adapters if needed,
    or kept for backward compatibility with static methods.
    """

    @staticmethod
    def save_field(polygon: Polygon, filename: str):
        """Guarda las coordenadas del polígono en un archivo JSON."""
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        coords = list(polygon.exterior.coords)
        data = {
            "type": "Polygon",
            "coordinates": coords
        }
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"Campo guardado en: {filename}")

    @staticmethod
    def load_field(filename: str) -> Polygon:
        """
        Legacy static method. 
        Uses the new Adapter internally to ensure consistency.
        """
        adapter = JSONMapAdapter(filename)
        return adapter.get_boundary()