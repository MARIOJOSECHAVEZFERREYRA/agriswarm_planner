from shapely.geometry import Point
import numpy as np

class MissionAnalyzer:
    """
    Analiza y compara misiones (Estatica vs Movil) y genera planes logisticos.
    """
    
    @staticmethod
    def simulate_static_mission(segmenter_class, polygon, raw_path, drone_specs, mobile_station_class):
        """
        Simula la misma mision pero con la estacion fija en el punto de inicio.
        Retorna: (cycles, total_dist_air, total_time)
        """
        # 1. Definir "Camino del Camion" como un punto estatico (buffer minimo en el inicio)
        p_start = raw_path[0]
        start_point = Point(p_start[:2])
        # Buffer minusculo para que shapely no llore, pero efectivamente es un punto
        static_region = start_point.buffer(0.1) 
        
        # 2. Instanciar un segmentador nuevo para no ensuciar el original
        # Asumimos que podemos instanciar uno nuevo con los specs
        segmenter = segmenter_class(drone_specs, mobile_station_class(truck_speed_mps=0.1)) # Velocidad irrelevante
        
        # 3. Correr segmentacion forzando el "truck_polygon" a ser el punto estatico
        cycles = segmenter.segment_path(polygon, raw_path, truck_polygon=static_region)
        
        # 4. Calcular metricas
        total_dist = 0.0
        total_time = 0.0
        
        for c in cycles:
            path = c['path']
            # Sumar distancias euclideas
            d = 0
            for i in range(len(path)-1):
                d += np.linalg.norm(np.array(path[i][:2]) - np.array(path[i+1][:2]))
            total_dist += d
            
        return cycles, total_dist

    @staticmethod
    def calculate_comprehensive_metrics(cycles, polygon, drone_specs):
        """
        Calcula metricas detalladas para el reporte:
        - Productividad (ha/h)
        - Dosis Real (L/ha)
        - Tiempos de Vuelo vs Muerto (min)
        """
        total_dist = 0
        spray_dist = 0
        deadhead_dist = 0
        
        # 1. Distancias
        for c in cycles:
            segments = c.get('segments', [])
            if segments:
                for s in segments:
                     d = np.linalg.norm(np.array(s['p1'][:2]) - np.array(s['p2'][:2]))
                     total_dist += d
                     if s['spraying']:
                         spray_dist += d
                     else:
                         deadhead_dist += d
            else:
                # Fallback simple
                path = c.get('path', [])
                for i in range(len(path)-1):
                    total_dist += np.linalg.norm(np.array(path[i][:2]) - np.array(path[i+1][:2]))

        # 2. Area
        area_m2 = polygon.area
        area_ha = area_m2 / 10000.0
        
        # 3. Tiempos
        # Usamos velocidad de trabajo para todo como aproximacion conservadora, 
        # o separamos si en el futuro tenemos velocity profiles.
        work_speed_ms = float(drone_specs.flight.work_speed_kmh.value) / 3.6
        flight_time_sec = total_dist / work_speed_ms
        flight_time_min = flight_time_sec / 60.0
        
        # Estimacion Tiempos Muertos de Recarga
        # Asumimos X min por parada? 
        reload_time_min = len(cycles) * 5.0 # 5 min por ciclo de recarga?
        total_op_time_min = flight_time_min + reload_time_min
        
        # 4. Productividad
        prod_ha_hr = 0
        if total_op_time_min > 0:
            prod_ha_hr = area_ha / (total_op_time_min / 60.0)
            
        # 5. Dosis
        # Total Litros = Spray Dist (m) * Swath (m) * (L/m2 ?) No...
        # Mejor: Dosis = (FlowRate * SprayTime) / Area ?
        # O mas simple: Usamos el tanque * ciclos? No exacto.
        # Usemos el rate configurado en el segmentador si es posible o inferirlo.
        # Dosis = (Volumen Total Aplicado) / (Area Cultivo)
        
        # Volumen Aplicado:
        # Sum of (Spray Dist / Speed) * FlowRate
        flow_l_min = float(drone_specs.spray.max_flow_l_min.value) # Asumimos max flow? O regulado?
        # En teoria el segmentador ajusta velocidad/flow para cumplir target.
        # Asumamos que aplicamos lo que requiere el area.
        # Pero podemos calcularlo "bottom-up":
        spray_time_min = (spray_dist / work_speed_ms) / 60.0
        total_vol_l = spray_time_min * flow_l_min # Esto asume flujo constante maximo.
        
        real_dosage = 0
        if area_ha > 0:
            real_dosage = total_vol_l / area_ha
            
        return {
            "area_ha": area_ha,
            "flight_time_min": flight_time_min,
            "total_op_time_min": total_op_time_min,
            "productivity_ha_hr": prod_ha_hr,
            "real_dosage_l_ha": real_dosage,
            "spray_dist_km": spray_dist / 1000.0,
            "dead_dist_km": deadhead_dist / 1000.0,
            "efficiency_ratio": (spray_dist / total_dist * 100) if total_dist else 0
        }

    @staticmethod
    def compare_missions(mobile_cycles, static_cycles):
        """
        Genera metricas comparativas.
        """
        def get_metrics(cycles):
            total_dist = 0
            flight_time = 0 
            deadhead_dist = 0
            spray_dist = 0
            truck_dist = 0
            
            for c in cycles:
                # Drone Path
                path = c['path']
                segments = c.get('segments', [])
                if segments:
                    for s in segments:
                         d = np.linalg.norm(np.array(s['p1'][:2]) - np.array(s['p2'][:2]))
                         total_dist += d
                         if s['spraying']:
                             spray_dist += d
                         else:
                             deadhead_dist += d
                
                # Truck Path
                t_path = c.get('truck_path_coords', [])
                if t_path and len(t_path) > 1:
                    for i in range(len(t_path)-1):
                        p1 = np.array(t_path[i][:2])
                        p2 = np.array(t_path[i+1][:2])
                        truck_dist += np.linalg.norm(p1 - p2)

            return total_dist, deadhead_dist, spray_dist, truck_dist

        m_total, m_dead, m_spray, m_truck = get_metrics(mobile_cycles)
        s_total, s_dead, s_spray, s_truck = get_metrics(static_cycles) # s_truck is usually 0
        
        # Ahorro
        dead_savings_km = (s_dead - m_dead) / 1000.0
        eff_improvement = 0.0
        if s_total > 0:
             # Eficiencia = Spray / Total
             eff_mobile = m_spray / m_total if m_total else 0
             eff_static = s_spray / s_total if s_total else 0
             eff_improvement = (eff_mobile - eff_static) * 100.0
             
        return {
            "mobile_dead_km": m_dead / 1000.0,
            "static_dead_km": s_dead / 1000.0,
            "mobile_truck_km": m_truck / 1000.0,
            "static_truck_km": s_truck / 1000.0,
            "savings_km": dead_savings_km, # Dron savings
            "efficiency_gain_pct": eff_improvement,
            "efficiency_static_pct": (s_spray / s_total * 100) if s_total else 0,
            "efficiency_mobile_pct": (m_spray / m_total * 100) if m_total else 0
        }

    @staticmethod
    def plan_logistics(mobile_cycles, drone_specs):
        """
        Genera el plan de recursos.
        """
        tank_l = float(drone_specs.spray.tank_l.value)
        
        # Estimacion de baterias
        # Tiempo de vuelo total vs Tiempo de carga
        # Esto es complejo, hagamos una heuristica simple propuesta por el usuario:
        # "Rotacion de carga". Necesitamos saber cuanto dura un vuelo y cuanto tarda en cargar.
        flight_time_min = 15.0 # Default
        charge_time_min = 30.0 # Default
        
        if drone_specs.flight.flight_time_min:
             flight_time_min = float(drone_specs.flight.flight_time_min['hover_loaded'].value) # Conservador
        
        if drone_specs.battery and drone_specs.battery.charge_time_min:
             charge_time_min = float(drone_specs.battery.charge_time_min.value)
             
        # Ratio: Si vuelo 10 min y carga 30 min, necesito 3 baterias cargando mientras vuelo 1.
        # Total packs = 1 (volando) + ceil(Charge / Flight)
        packs_needed = 1 + int(np.ceil(charge_time_min / flight_time_min))
        
        # Tabla de paradas
        stops = []
        total_liter_mix = 0.0
        
        # Velocidad de trabajo para calculos
        work_speed_ms = float(drone_specs.flight.work_speed_kmh.value) / 3.6
        flow_l_min = float(drone_specs.spray.max_flow_l_min.value)
        
        for i, cycle in enumerate(mobile_cycles):
            stop_type = "Inicio" if i == 0 else f"Parada {i}"
            
            # --- CALCULO PRECISO DE CONSUMO ---
            # Sumar distancia de segmentos con spraying=True
            segments = cycle.get('segments', [])
            spray_dist_m = 0
            if segments:
                for s in segments:
                    if s['spraying']:
                        d = np.linalg.norm(np.array(s['p1'][:2]) - np.array(s['p2'][:2]))
                        spray_dist_m += d
            else:
                # Fallback: asumir todo el camino es spray (peor caso)
                path = cycle['path']
                for j in range(len(path)-1):
                    spray_dist_m += np.linalg.norm(np.array(path[j][:2]) - np.array(path[j+1][:2]))

            # Tiempo rociando (min)
            spray_time_min = (spray_dist_m / work_speed_ms) / 60.0
            
            # Litros consumidos
            # Asumiendo flujo constante:
            liters_consumed = spray_time_min * flow_l_min
            
            # Clamp al tamaño del tanque (por si acaso)
            if liters_consumed > tank_l: liters_consumed = tank_l
            
            total_liter_mix += liters_consumed
            
            # Generar descripcion accionable
            # "Cambio Batería + Recarga 15.4L"
            action_desc = f"Cambio Bat. + Cargar {liters_consumed:.1f}L"
            if i == 0:
                action_desc = f"Llenar Tanque ({liters_consumed:.1f}L)"
            
            stops.append({
                "name": stop_type,
                "action": action_desc,
                "notes": f"Cobertura: {spray_dist_m:.0f}m" 
            })
            
        return {
            "battery_packs": packs_needed,
            "total_mix_l": total_liter_mix, 
            "stops": stops
        }
