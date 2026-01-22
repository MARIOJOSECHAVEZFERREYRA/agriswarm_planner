import numpy as np
from shapely.geometry import Polygon

class MarginReducer:
    """
    Implementation of Phase 1: Boundary Shrinking.
    Based on Equations 1, 2, and 3 of the paper by Li et al. (2023).
    
    
    """

    @staticmethod
    def shrink(polygon: Polygon, margin_h: float) -> Polygon:
        """
        Contracts a polygon by a distance 'h' towards its interior.
        
        :param polygon: Original shapely Polygon.
        :param margin_h: Safety distance in meters (h).
        :return: New reduced Polygon.
        """
        # 1. Ensure Counter-Clockwise (CCW) orientation for mathematical consistency
        # In Shapely/GIS, CCW means the interior is on the left.
        # 
        if polygon.exterior.is_ccw:
            coords = np.array(polygon.exterior.coords)[:-1] # Remove the last repeated point
        else:
            coords = np.array(polygon.exterior.coords)[::-1][:-1]

        num_points = len(coords)
        new_coords = []

        for i in range(num_points):
            # Get vertices: Previous (prev), Current (curr), Next (next)
            prev_p = coords[i - 1]
            curr_p = coords[i]
            next_p = coords[(i + 1) % num_points]

            # --- EQUATION 1: Bisector Vector Calculation ---
            # Vectors pointing from current vertex to neighbors
            vec_prev = prev_p - curr_p
            vec_next = next_p - curr_p

            # Normalize vectors (make them unit vectors)
            len_prev = np.linalg.norm(vec_prev)
            len_next = np.linalg.norm(vec_next)
            
            # Avoid division by zero if there are duplicate points
            if len_prev == 0 or len_next == 0:
                new_coords.append(curr_p)
                continue

            u_prev = vec_prev / len_prev
            u_next = vec_next / len_next

            # Sum Vector (Bisector Direction)
            # Note: Points towards the "interior" of the angle formed by the lines
            # 
            vec_C = u_prev + u_next
            
            # --- EQUATION 2 and Concavity Detection ---
            # Calculate interior angle theta using dot product
            # Dot product: a . b = |a||b| cos(theta)
            dot_prod = np.dot(u_prev, u_next)
            # Clip value to avoid numerical errors in arccos (e.g., 1.00000001)
            dot_prod = np.clip(dot_prod, -1.0, 1.0)
            theta = np.arccos(dot_prod)

            # Detect if angle is Concave (Reflex) using Cross Product (2D)
            # For 3D, we use only X and Y components to determine concavity (projection)
            # Cross product 2D: a_x*b_y - a_y*b_x
            cross_prod_2d = u_next[0] * u_prev[1] - u_next[1] * u_prev[0]
            
            # Determine if convex or concave
            # In CCW traversal, if cross > 0 the turn is to the left (standard convex)
            # If cross < 0, it is a turn to the right ("bite" inwards -> concave)
            # 
            is_convex = cross_prod_2d > 0

            # --- EQUATION 3: Offset Magnitude ---
            # Distance to move the vertex: L = h / sin(theta/2)
            # Note: theta calculated by arccos is always [0, pi], which is the internal angle
            # (or external depending on reference), but for magnitude it works.
            # However, if vectors are collinear (theta=180), sin(90)=1 -> L=h (correct)
            # If theta=0 (needle/sharp corner), sin(0)=0 -> L=infinity (correct, cannot shrink a needle)
            # 
            
            sin_half_theta = np.sin(theta / 2.0)
            
            if sin_half_theta < 1e-6:
                # Degenerate case (straight line or very sharp needle), use h directly or skip
                offset_magnitude = margin_h
            else:
                offset_magnitude = margin_h / sin_half_theta

            # --- Final Displacement Direction ---
            norm_C = np.linalg.norm(vec_C)
            if norm_C < 1e-6:
                # Special case: opposite vectors (180 degrees). The bisector is perpendicular.
                # Rotate u_next 90 degrees to the left (CCW)
                dir_vector = np.array([-u_next[1], u_next[0]])
            else:
                dir_vector = vec_C / norm_C

            # APPLY THE PAPER:
            # "Concave point has the opposite shift direction"
            # If convex, vec_C points towards the inside of the polygon.
            # If concave, vec_C points towards the outside ("pacman mouth"), 
            # but we want to shrink the polygon, so we must move the boundary "into the meat".
            # At a concave point, moving inwards means going AGAINST vec_C.
            # 
            
            if is_convex:
                final_movement = dir_vector * offset_magnitude
            else:
                # Invert direction for concave points
                # Note: Depending on exact geometry, sometimes the sum vector already points
                # outwards. We verify visually:
                # Convex (V): Sum points in. We want to go in. OK.
                # Concave (Internal L): Sum points to void. We want to go to meat (in).
                # Therefore, we need to invert.
                final_movement = -dir_vector * offset_magnitude

            new_p = curr_p + final_movement
            new_coords.append(new_p)

        # Close the polygon and return
        if len(new_coords) < 3:
            return polygon # Failure, return original or empty
            
        return Polygon(new_coords)