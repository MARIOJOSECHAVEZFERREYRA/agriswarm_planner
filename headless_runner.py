import sys
import os
import numpy as np
from shapely.geometry import Polygon

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from algorithms import BoustrophedonPlanner, GeneticOptimizer, MarginReducer

from algorithms.map_interface import MapInterface
from src.field_io import JSONMapAdapter # Importing from valid location

def main():
    print("Running Headless Planner Verification (3D Support)...")
    
    # 1. Load 3D Map
    map_file = os.path.join(os.path.dirname(__file__), 'data', 'test_map_3d.json')
    if not os.path.exists(map_file):
        print(f"Test map not found: {map_file}")
        return
        
    adapter = JSONMapAdapter(map_file)
    poly = adapter.get_boundary()
    
    print(f"Polygon Area: {poly.area} m2")
    if poly.has_z:
        print("SUCCESS: 3D Coordinates detected in Polygon.")
        # Print a sample coordinate
        print(f"Sample Coord: {list(poly.exterior.coords)[2]}") # Should show Z
    else:
        print("WARNING: Polygon is 2D.")

    # 2. Margin Reduction
    print("Applying Safety Margin...")
    # MarginReducer might strip Z if it creates new points strictly from X,Y.
    # Let's see.
    safe_poly = MarginReducer.shrink(poly, margin_h=2.0)
    print(f"Safe Polygon Area: {safe_poly.area} m2")
    
    if safe_poly.is_empty:
        print("Error: Polygon became empty after margin reduction.")
        return

    # 3. Path Planning
    print("Planning Path...")
    planner = BoustrophedonPlanner(spray_width=5.0)
    optimizer = GeneticOptimizer(planner, pop_size=10, generations=5)
    
    best_angle, path, metrics = optimizer.optimize(safe_poly)
    
    print("Optimization Complete.")
    print(f"Best Angle: {best_angle:.2f}")
    print(f"Path Waypoints: {len(path)}")
    
    if path:
        z_vals = [p[2] for p in path if len(p) > 2]
        if z_vals:
             print(f"SUCCESS: Generated path contains Z coordinates. (Avg Z: {np.mean(z_vals):.1f})")
        else:
             print("NOTICE: Generated path is 2D (Z lost during processing).")
    else:
        print("FAILURE: No path generated.")

    # 4. Concave Decomposition Test
    print("\n--- Testing Concave Decomposition (U-Shape) ---")
    # U-shape polygon
    # (0,100) -> (100,100) -> (100,0) -> (80,0) -> (80,80) -> (20,80) -> (20,0) -> (0,0)
    u_coords = [
        (0,100), (100,100), (100,0), (80,0), 
        (80,80), (20,80), (20,0), (0,0)
    ]
    u_poly = Polygon(u_coords)
    print(f"U-Shape Area: {u_poly.area}")
    
    # We verify if the optimizer runs without error and generates a path covering the area
    # Note: Decomposition happens inside optimize method now
    best_angle_u, path_u, metrics_u = optimizer.optimize(u_poly)
    
    print(f"Concave Optimization Complete.")
    print(f"Best Angle: {best_angle_u:.2f}")
    print(f"Path Elements: {len(path_u)}")
    # Validating if path covers the shape properly is complex to auto-verify perfectly 
    # but we check if we have enough waypoints for a complex shape
    if len(path_u) > 10:
        print("SUCCESS: Concave path generated with multiple waypoints.")
    else:
        print("WARNING: Path seems too short for a U-shape.")

if __name__ == "__main__":
    main()
