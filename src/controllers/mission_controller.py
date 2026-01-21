from datetime import datetime
from shapely.geometry import Polygon, LineString, MultiPolygon, Point
from algorithms.strategy import StrategyFactory
from algorithms.margin import MarginReducer
from algorithms.segmentation import MissionSegmenter
from algorithms.mobile_station import MobileStation
from data import DroneDB

class MissionController:
    """
    Controller responsible for orchestrating the mission planning process.
    Handles geometry validation, spec management, optimization, and segmentation.
    Uses Strategy Pattern for optimization logic.
    """
    
    def __init__(self):
        self.last_result = None

    def run_mission_planning(self, polygon_points, drone_name, overrides, 
                             truck_route_points=None, truck_offset=0.0, 
                             use_mobile_station=True, strategy_name="genetic"):
        """
        Executes the full mission planning workflow.
        
        Args:
            polygon_points (list): List of (x, y) tuples for the field boundary.
            drone_name (str): Name of the drone model.
            overrides (dict): Dictionary of UI overrides (swath, tank, speed, etc.).
            truck_route_points (list, optional): List of (x, y) for manual truck route.
            truck_offset (float): Offset for truck route snapping.
            use_mobile_station (bool): Whether to calculate mobile station logistics.
            strategy_name (str): optimization strategy ("genetic" or "simple").
            
        Returns:
            dict: Mission results containing geometry, cycles, metrics, and compatibility info.
        """
        
        # 1. Geometry Validation
        if len(polygon_points) < 3:
            raise ValueError("Polygon must have at least 3 points.")
            
        polygon = Polygon(polygon_points)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
            
        # Sanitize Polygon (Fix side location conflicts & micro-segments)
        try:
            if not polygon.is_valid:
                cleaned = polygon.buffer(0)
                if cleaned.geom_type == 'MultiPolygon':
                    # Keep largest area (Filter out noise/islands)
                    cleaned = max(cleaned.geoms, key=lambda p: p.area)
                polygon = cleaned
            
            # Simplify to remove micro-segments (<1cm) which cause topology exceptions
            polygon = polygon.simplify(0.01, preserve_topology=True)
            
            # Final check
            if not polygon.is_valid:
                 polygon = polygon.buffer(0)
        except Exception as e:
            print(f"Warning: Error sanitizing polygon: {e}")

        # 2. Spec Management (Overrides)
        import copy
        specs = copy.deepcopy(DroneDB.get_specs(drone_name))
        
        # Apply Overrides
        if 'tank' in overrides and specs.spray:
            specs.spray.tank_l.value = overrides['tank']
            
        if 'speed' in overrides and specs.flight:
            specs.flight.work_speed_kmh.value = overrides['speed'] * 3.6
            
        real_swath = overrides.get('swath', 5.0)
        if not 'swath' in overrides and specs.spray and specs.spray.swath_m:
            min_s = float(specs.spray.swath_m[0].value)
            max_s = float(specs.spray.swath_m[1].value)
            real_swath = (min_s + max_s) / 2.0

        # 3. Safety Margin
        margin_h = DroneDB.calculate_safety_margin_m(specs, buffer_gps=0.5)
        
        try:
            safe_polygon = MarginReducer.shrink(polygon, margin_h=margin_h)
        except Exception:
            raise ValueError("Field too small for safety margin.")

        # 4. Truck Route Processing
        truck_route_line = None
        
        # Smart Snap Logic
        if truck_route_points and len(truck_route_points) >= 2:
            if truck_offset > 0.1:
                try:
                    # Create buffered shell from the field boundary
                    field_shell = polygon.buffer(truck_offset, join_style=2)
                    shell_linear = field_shell.exterior
                    
                    snapped_points = []
                    for pt in truck_route_points:
                        p_obj = Point(pt)
                        # Find nearest point on shell (projection)
                        proj_dist = shell_linear.project(p_obj)
                        snapped_pt = shell_linear.interpolate(proj_dist)
                        snapped_points.append((snapped_pt.x, snapped_pt.y))
                    
                    truck_route_line = LineString(snapped_points)
                except Exception as e:
                    print(f"Snap failed: {e}. Using raw points.")
                    truck_route_line = LineString(truck_route_points)
            else:
                truck_route_line = LineString(truck_route_points)

        # 5. Route Optimization (STRATEGY PATTERN)
        optimizer = StrategyFactory.get_strategy(strategy_name)
        print(f"Optimization Strategy: {strategy_name}")
        
        opt_result = optimizer.optimize(
            safe_polygon,
            swath_width=real_swath, 
            truck_route=truck_route_line
        )
        
        best_angle = opt_result['angle']
        best_path = LineString(opt_result['path']) if opt_result['path'] else None
        
        if not best_path:
             raise ValueError("Could not generate flight path with current settings.")

        # 6. Mission Segmentation (Logistics)
        # Calculate max flow based on overrides
        app_rate = overrides.get('app_rate', 10.0)
        speed_ms = overrides.get('speed', 5.0)
        speed_kmh = speed_ms * 3.6
        calc_flow_l_min = (app_rate * speed_kmh * real_swath) / 600.0
        
        # Patch specs flow for segmenter
        if specs.spray:
            specs.spray.max_flow_l_min.value = calc_flow_l_min

        # Run Segmentation (Mobile vs Static)
        
        # A. Mobile Calculation
        mission_cycles = []
        full_metrics = {}
        comparison_metrics = {}
        resource_data = {}
        
        if use_mobile_station:
            # Helper for metrics
            def calculate_metrics(cycles):
                truck_km = sum(c.get('truck_dist', 0) for c in cycles) / 1000.0
                # Approximate efficiency (needs deep segment analysis)
                return {
                    "truck_dist_km": truck_km,
                    "dead_dist_km": 0.0,
                    "efficiency_ratio": 0.8, # Mock
                    "total_energy_Wh": 0.0,
                    "total_mix_l": 0.0,
                    "stops": len(cycles)
                }

            # A. Mobile Calculation
            station_speed = 5.0 # Default truck speed
            station = MobileStation(truck_speed_mps=station_speed)
            segmenter = MissionSegmenter(specs, station, target_rate_l_ha=app_rate, work_speed_kmh=speed_kmh)
            
            mission_cycles = segmenter.segment_path(
                polygon=safe_polygon, 
                raw_path=list(best_path.coords),
                truck_polygon=safe_polygon,
                truck_route_line=truck_route_line
            )
            full_metrics = calculate_metrics(mission_cycles)
            
            # B. Static Calculation (for comparison)
            if truck_route_line:
                home_point = truck_route_line.coords[0]
            else:
                home_point = best_path.coords[0]
                
            static_station = MobileStation(truck_speed_mps=0)
            static_segmenter = MissionSegmenter(specs, static_station, target_rate_l_ha=app_rate, work_speed_kmh=speed_kmh)
            
            static_cycles = static_segmenter.segment_path(
                polygon=safe_polygon,
                raw_path=list(best_path.coords),
                start_point=home_point
            )
            static_metrics = calculate_metrics(static_cycles)
            
            # 7. Generate Comparison Data
            comparison_metrics = {
                "static_dead_km": static_metrics['dead_dist_km'],
                "mobile_dead_km": full_metrics['dead_dist_km'],
                "savings_km": static_metrics['dead_dist_km'] - full_metrics['dead_dist_km'],
                
                "efficiency_static_pct": static_metrics['efficiency_ratio'],
                "efficiency_mobile_pct": full_metrics['efficiency_ratio'],
                "efficiency_gain_pct": full_metrics['efficiency_ratio'] - static_metrics['efficiency_ratio'],
                
                "static_truck_km": 0.0,
                "mobile_truck_km": full_metrics.get('truck_dist_km', 0.0)
            }
            
            # Resource Estimation
            # Battery: Total Energy / Pack Capacity
            total_energy = full_metrics.get('total_energy_Wh', 0)
            pack_cap = 0
            if specs.battery:
                if specs.battery.energy_wh:
                    pack_cap = float(specs.battery.energy_wh.value)
                elif specs.battery.capacity_ah and specs.battery.voltage_v:
                    pack_cap = float(specs.battery.capacity_ah.value) * float(specs.battery.voltage_v.value)
            
            if pack_cap > 0:
                packs_req = (total_energy / (pack_cap * 0.8)) # 80% DOD
            else:
                packs_req = 0
            import math
            packs_req = math.ceil(packs_req)
            
            resource_data = {
                "total_mix_l": full_metrics.get('total_mix_l', 0),
                "battery_packs": packs_req,
                "stops": full_metrics.get('stops', [])
            }

        # Pack results
        return {
            "polygon": polygon,
            "safe_polygon": safe_polygon,
            "mission_cycles": mission_cycles,
            "truck_route_line": truck_route_line,
            "metrics": full_metrics,
            "comparison": comparison_metrics,
            "resources": resource_data,
            "best_angle": best_angle
        }
