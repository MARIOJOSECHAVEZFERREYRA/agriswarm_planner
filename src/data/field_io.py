import json
import os
from shapely.geometry import Polygon

class FieldIO:
    """
    Module to save and load agricultural fields in simple JSON format.
    """

    @staticmethod
    def save_field(polygon: Polygon, filename: str):
        """Saves the polygon coordinates to a JSON file."""
        # Ensure the directory exists
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        # Extract coordinates as a list of tuples
        coords = list(polygon.exterior.coords)
        
        data = {
            "type": "Polygon",
            "coordinates": coords
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"Field saved to: {filename}")

    @staticmethod
    def load_field(filename: str) -> Polygon:
        """Loads a polygon from a JSON file."""
        if not os.path.exists(filename):
            raise FileNotFoundError(f"File not found: {filename}")
            
        with open(filename, 'r') as f:
            data = json.load(f)
            
        return Polygon(data["coordinates"])