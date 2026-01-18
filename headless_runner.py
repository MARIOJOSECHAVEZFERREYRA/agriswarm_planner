import sys
import os
import numpy as np
from shapely.geometry import Polygon

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from algorithms import BoustrophedonPlanner, GeneticOptimizer, MarginReducer, MobileStation, MissionSegmenter

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

    # 5. Truck Synergy Test
    print("\n--- Testing Truck-Drone Synergy ---")
    mobile_station = MobileStation(truck_speed_mps=5.0) # 5 m/s
    
    # Assume Drone finishes at the last point of the generated path
    if path_u and len(path_u) > 0:
        p_exit = path_u[-1]
        p_exit_2d = (p_exit[0], p_exit[1]) # Shapely uses 2D
        
        # Truck starts at index 0 (0,100)
        truck_start = u_coords[0]
        
        r_opt, dist, time_s = mobile_station.calculate_rendezvous(u_poly, p_exit_2d, truck_start)
        
        print(f"Drone Exit Point: {p_exit_2d}")
        print(f"Truck Start: {truck_start}")
        print(f"Optimal Rendezvous (R_opt): ({r_opt.x:.2f}, {r_opt.y:.2f})")
        print(f"Truck Travel Distance: {dist:.2f} m")
        print(f"Truck Travel Time: {time_s:.1f} s")
        
        # Verify Feasibility
        # Assume Drone has 15 mins (900s) left
        is_feasible = mobile_station.check_feasibility(time_s, 900)
        if is_feasible:
             print("SUCCESS: Truck arrives before drone battery depletion.")
        else:
             print("FAILURE: Truck is too slow!")

    # 6. Mission Segmentation Test (Physics)
    print("\n--- Testing Physics-Based Segmentation (Capacity Check) ---")
    # Using a Mock Drone Spec
    class MockSpec:
        pass
    
    mock_spec = MockSpec()
    mock_spec.flight = MockSpec()
    mock_spec.flight.work_speed_kmh = type('obj', (object,), {'value': 20.0})
    mock_spec.flight.flight_time_min = {'hover_loaded': type('obj', (object,), {'value': 10.0})} # Short battery for testing split
    
    mock_spec.spray = MockSpec()
    mock_spec.spray.swath_m = [type('obj', (object,), {'value': 4.0}), type('obj', (object,), {'value': 6.0})]
    mock_spec.spray.tank_l = type('obj', (object,), {'value': 10.0}) # Small tank
    mock_spec.spray.max_flow_l_min = type('obj', (object,), {'value': 5.0})

    segmenter = MissionSegmenter(mock_spec, mobile_station, target_rate_l_ha=20.0)
    
    # Check pump
    ok, msg, flow = segmenter.validate_pump()
    print(f"Pump Check: {msg} (Flow: {flow:.2f} L/min)")
    
    # Run segmentation on the U-Shape path
    if path_u:
        cycles = segmenter.segment_path(u_poly, path_u)
        print(f"Total Cycles Generated: {len(cycles)}")
        for i, cycle in enumerate(cycles):
            print(f"  Cycle {i+1}: {len(cycle['path'])} waypoints. Truck Dist: {cycle['truck_dist']:.1f} m")
            
        if len(cycles) > 1:
            print("SUCCESS: Path segmented into multiple cycles due to resource constraints.")
        else:
            print("NOTICE: Single cycle sufficient (or constraints too loose).")

if __name__ == "__main__":
    main()
