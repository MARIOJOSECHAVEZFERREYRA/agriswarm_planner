import numpy as np
from shapely.geometry import Polygon, LineString, MultiLineString, Point
from shapely import affinity
from typing import List, Tuple

class BoustrophedonPlanner:
    """
    Implementation of Phase 3: Boustrophedon (Zig-Zag) Path Generation.
    Adapted to return fitness metrics according to Li et al. (2023).
    
    
    """

    def __init__(self, spray_width: float = 5.0):
        """
        :param spray_width: Effective spray width (d) in meters.
        
        """
        self.spray_width = spray_width

    def generate_path(self, polygon: Polygon, angle_deg: float) -> Tuple[List[tuple], float, float]:
        """
        Generates a coverage path for a given angle and calculates metrics.
        
        :param polygon: Work zone polygon (must be convex or a sub-zone).
        :param angle_deg: Sweep angle (heading) in degrees.
        :return: (waypoints, flight_distance_l, coverage_area_S_prime)
        """
        # 1. Rotate the polygon to align the sweep with the horizontal X-axis
        # We use the centroid to rotate and then un-rotate without losing position
        # 
        centroid = polygon.centroid
        rotated_poly = affinity.rotate(polygon, -angle_deg, origin=centroid)
        
        min_x, min_y, max_x, max_y = rotated_poly.bounds
        
        # Generate sweep lines
        lines = []
        y_current = min_y + (self.spray_width / 2)
        direction = True # True = Left -> Right
        
        # Internal metrics (in the rotated system)
        total_spray_length = 0.0
        
        while y_current < max_y:
            # Infinite sweep line
            sweepline = LineString([(min_x - 1000, y_current), (max_x + 1000, y_current)])
            intersection = sweepline.intersection(rotated_poly)
            
            if not intersection.is_empty:
                # Handle complex geometries (MultiLineString)
                if isinstance(intersection, MultiLineString):
                    segs = list(intersection.geoms)
                else:
                    segs = [intersection]
                
                # Sort segments by X coordinate (always left to right first)
                segs.sort(key=lambda s: s.coords[0][0])

                for seg in segs:
                    coords = list(seg.coords)
                    
                    # Calculate spray length (for S')
                    # According to Eq. 13: S' = Sum(length * d)
                    seg_len = Point(coords[0]).distance(Point(coords[-1]))
                    total_spray_length += seg_len
                    
                    # Implement Zig-Zag (reverse direction if needed)
                    if not direction:
                        coords.reverse()
                    
                    lines.append(coords)
            
            y_current += self.spray_width
            direction = not direction # Change direction for the next line

        # 2. Build Continuous Path (Join segments)
        # This is vital to calculate 'l' (actual flight distance including turns)
        continuous_path_rotated = []
        if not lines:
            return [], 0.0, 0.0

        for i in range(len(lines)):
            segment = lines[i]
            # If it is not the first segment, add connection from the previous one
            if i > 0:
                prev_end = lines[i-1][-1]
                curr_start = segment[0]
                # Here we could add smooth turn logic (Dubins), 
                # but for now we use direct connection (straight line)
                # The paper assumes Euclidean distance for 'l' (Eq. 11)
                # 
            
            continuous_path_rotated.extend(segment)

        # 3. Un-rotate the complete path to return to GPS/Real coordinates
        final_waypoints = []
        flight_distance_l = 0.0
        
        # Convert list of points to LineString to rotate it all at once (more efficient)
        if len(continuous_path_rotated) > 1:
            path_line = LineString(continuous_path_rotated)
            restored_path = affinity.rotate(path_line, angle_deg, origin=centroid)
            
            # Extract coordinates
            final_waypoints = list(restored_path.coords)
            
            # Calculate 'l' (Total Flight Distance) - Eq. 11
            # flight_distance_l = restored_path.length 
            # (Shapely calculates Euclidean geodesic length correctly)
            flight_distance_l = restored_path.length
            
        elif len(continuous_path_rotated) == 1:
            # Edge case: a single point
            p = Point(continuous_path_rotated[0])
            restored_p = affinity.rotate(p, angle_deg, origin=centroid)
            final_waypoints = [restored_p.coords[0]]
            flight_distance_l = 0.0

        # 4. Calculate S' (Estimated Coverage Area) - Eq. 13
        # S' = Total spray line length * Spray width
        coverage_area_s_prime = total_spray_length * self.spray_width

        return final_waypoints, flight_distance_l, coverage_area_s_prime