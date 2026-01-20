import numpy as np
import random
from shapely.geometry import Polygon, LineString, Point
from typing import List, Tuple, Optional
from .path_planner import BoustrophedonPlanner
from .cost_evaluator import RouteCostEvaluator
from .decomposition import ConcaveDecomposer

class GeneticOptimizer:
    """
    Implementación de la Fase 4: Optimización por Algoritmo Genético (GA).
    Basado en el Algoritmo 1 y Ecuaciones 11-17 del paper de Li et al. (2023).
    """

    def __init__(self, planner: BoustrophedonPlanner, 
                 pop_size=200, generations=300, 
                 crossover_rate=0.4, mutation_rate=0.001):
        """
        Inicialización con parámetros de la Tabla 1 del paper.
        """
        self.planner = planner
        self.pop_size = pop_size
        self.generations = generations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        
        # El paper usa 3 decimales de precisión 
        self.precision_decimals = 3 

    def optimize(self, polygon: Polygon, truck_route: Optional[LineString] = None) -> Tuple[float, List[tuple], dict]:
        """
        Ejecuta el ciclo evolutivo completo para encontrar el ángulo óptimo.
        :param truck_route: Ruta lineal opcional para optimizar el punto de encuentro.
        :return: (best_angle, best_path, metrics)
        """
        # 1. Inicialización de Población (Ángulos aleatorios 0-360) [cite: 777]
        population = [random.uniform(0, 360) for _ in range(self.pop_size)]
        
        best_solution = None
        best_fitness = -1.0
        
        # Área objetivo (S) del polígono [cite: 792]
        target_area_S = polygon.area

        print(f"Iniciando Algoritmo Genetico ({self.generations} generaciones)...")

        for gen in range(self.generations):
            fitness_values = []
            metrics_list = []
            
            # --- EVALUACIÓN DE LA POBLACIÓN ---
            distances_l = []  # Para almacenar todas las distancias 'l' de esta gen
            logistics_costs = [] # Costos logisticos (distancia a truck_route - Anchor)
            coop_costs = []      # Costos cooperativos (distancia perimetral entre partes)
            
            raw_metrics = []  # (l, S_prime, path, log_cost, coop_cost) para cada individuo

            for angle in population:
                # 1. Descomposición del Polígono (Fase 2)
                sub_polygons = ConcaveDecomposer.decompose(polygon, angle)
                
                # 2. Generación de Ruta para cada sub-polígono (Fase 3)
                total_path = []
                sub_paths = [] # Almacenar partes separadas para costo cooperativo
                total_l = 0.0
                total_s_prime = 0.0
                
                for sub_poly in sub_polygons:
                    path, l, s_prime = self.planner.generate_path(sub_poly, angle)
                    total_path.extend(path)
                    sub_paths.append(path)
                    total_l += l
                    total_s_prime += s_prime
                
                # COOPERATIVE COST: Distancia perimetral del camion entre sub-rutas
                truck_perimeter_cost = RouteCostEvaluator.calculate_total_truck_cost(polygon, sub_paths)
                
                # Logistics Cost Calc (Custom Route Anchor)
                log_cost = 0.0
                if truck_route and len(total_path) > 1:
                    # Penalizar si Start/End están lejos de la ruta del camión
                    p_start = Point(total_path[0])
                    p_end = Point(total_path[-1])
                    d1 = truck_route.distance(p_start)
                    d2 = truck_route.distance(p_end)
                    log_cost = d1 + d2 
                
                distances_l.append(total_l)
                logistics_costs.append(log_cost)
                coop_costs.append(truck_perimeter_cost)
                
                raw_metrics.append((total_l, total_s_prime, total_path, log_cost, truck_perimeter_cost))

            # --- CÁLCULO DE FITNESS (Ec. 14 y 15) ---
            # Normalización
            sum_sq_l = sum(d**2 for d in distances_l)
            sqrt_sum_sq_l = np.sqrt(sum_sq_l) if sum_sq_l > 0 else 1.0
            
            sum_sq_log = sum(d**2 for d in logistics_costs)
            sqrt_sum_log = np.sqrt(sum_sq_log) if sum_sq_log > 0 else 1.0
            
            sum_sq_coop = sum(d**2 for d in coop_costs)
            sqrt_sum_coop = np.sqrt(sum_sq_coop) if sum_sq_coop > 0 else 1.0

            for i, (l, s_prime, path, log_cost, coop_cost) in enumerate(raw_metrics):
                # 1. Distancia Normalizada (Ec. 14) [cite: 800]
                i_norm = l / sqrt_sum_sq_l
                
                # 2. Error de Cobertura
                coverage_error = abs(s_prime - target_area_S) / target_area_S if target_area_S > 0 else 0
                
                # 3. Costo Logistico Normalizado (Anchor/Custom Route)
                log_norm = (log_cost / sqrt_sum_log) if truck_route else 0.0
                
                # 4. Costo Cooperativo Normalizado (Truck Perimeter)
                coop_norm = coop_cost / sqrt_sum_coop
                
                # 5. Función de Fitness INTEGRADA
                # Pesos:
                # w_log (Anchor) = 5.0 (Prioridad Maxima a ruta manual)
                # w_coop (Perimetro) = 2.0 (Evitar saltos de rana)
                
                w_log = 5.0 if truck_route else 0.0 # Increased to prioritize alignment
                w_coop = 2.0
                
                denom = i_norm + coverage_error + (w_log * log_norm) + (w_coop * coop_norm)
                fitness = 1.0 / denom if denom > 0 else 0.0
                
                fitness_values.append(fitness)
                
                # Guardar métricas completas
                metrics = {
                    "angle": population[i],
                    "fitness": fitness,
                    "l": l,
                    "s_prime": s_prime,
                    "eta": coverage_error * 100, # En porcentaje
                    "path": path,
                    "truck_cost": coop_cost, # NEW: Truck Perimeter Distance
                    "anchor_cost": log_cost  # NEW: Distance to Custom Route
                }
                metrics_list.append(metrics)

                # Actualizar el mejor global
                if fitness > best_fitness:
                    best_fitness = fitness
                    best_solution = metrics

            # --- SELECCIÓN, CRUCE Y MUTACIÓN (Generar nueva población) ---
            new_population = []
            
            # Elitismo: Mantener siempre el mejor individuo de la historia (Opcional pero recomendado)
            new_population.append(best_solution["angle"])
            
            while len(new_population) < self.pop_size:
                # 1. Selección por Ruleta (Ec. 16-17) [cite: 811]
                parent1 = self._roulette_selection(population, fitness_values)
                parent2 = self._roulette_selection(population, fitness_values)
                
                # 2. Cruce (Crossover) [cite: 821-823]
                if random.random() < self.crossover_rate:
                    child1, child2 = self._crossover(parent1, parent2)
                else:
                    child1, child2 = parent1, parent2
                
                # 3. Mutación [cite: 926-929]
                child1 = self._mutate(child1)
                child2 = self._mutate(child2)
                
                new_population.append(child1)
                if len(new_population) < self.pop_size:
                    new_population.append(child2)
            
            population = new_population

            # Log cada 50 generaciones
            if (gen + 1) % 50 == 0:
                print(f"   Gen {gen+1}/{self.generations} | Mejor Fitness: {best_fitness:.4f} | Ángulo: {best_solution['angle']:.2f}°")

        return best_solution["angle"], best_solution["path"], best_solution

    def _roulette_selection(self, population, fitness_values):
        """
        Selección de Ruleta (Ec. 16 y 17).
        La probabilidad de selección es proporcional al fitness.
        """
        total_fitness = sum(fitness_values)
        if total_fitness == 0:
            return random.choice(population)
        
        pick = random.uniform(0, total_fitness)
        current = 0
        for i, angle in enumerate(population):
            current += fitness_values[i]
            if current > pick:
                return angle
        return population[-1]

    def _crossover(self, p1, p2):
        """
        Operador de Cruce.
        El paper usa binario, pero para implementación continua usamos promedio ponderado
        o intercambio simple para simular la mezcla genética.
        Aquí usamos cruce aritmético simple.
        """
        alpha = random.random()
        c1 = alpha * p1 + (1 - alpha) * p2
        c2 = (1 - alpha) * p1 + alpha * p2
        return c1 % 360, c2 % 360

    def _mutate(self, angle):
        """
        Operador de Mutación.
        Introduce una variación aleatoria pequeña para escapar de mínimos locales.
        """
        if random.random() < self.mutation_rate:
            # Mutación gaussiana pequeña
            mutation_amount = random.gauss(0, 10) # Std dev de 10 grados
            return (angle + mutation_amount) % 360
        return angle