from abc import ABC, abstractmethod
from typing import List, Optional
import numpy as np
from shapely.geometry import Polygon

class MapInterface(ABC):
    """
    Abstract Interface for Map Data Sources.
    Allows algorithms to be agnostic of the data source (JSON, ROS, Database).
    """

    @abstractmethod
    def get_boundary(self) -> Polygon:
        """
        Returns the main field boundary as a Polygon.
        """
        pass

    @abstractmethod
    def get_obstacles(self) -> List[Polygon]:
        """
        Returns a list of obstacle polygons within the field.
        """
        pass
    
    @abstractmethod
    def get_metadata(self) -> dict:
        """
        Returns arbitrary metadata (name, location, etc.)
        """
        pass
