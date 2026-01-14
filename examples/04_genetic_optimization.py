import sys
import os
import matplotlib.pyplot as plt

# Setup path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from geometry import FieldGenerator
from margin import MarginReducer
from path_planner import BoustrophedonPlanner
from genetic_optimizer import GeneticOptimizer

def main():
    # 1. Configuración del Escenario
    print("--- AgriSwarm: Optimización Genética (Li et al. 2023) ---")
    
    # Campo en L (difícil para algoritmos simples)
    original_field = FieldGenerator.create_l_shape_field()
    
    # 2. Fase 1: Reducción de Márgenes (Ec. 1-3)
    # Reducimos 2 metros para seguridad
    safe_field = MarginReducer.shrink(original_field, margin_h=2.0)
    
    # 3. Preparar Herramientas
    planner = BoustrophedonPlanner(spray_width=5.0)
    
    # 4. Fase 4: Optimización Genética (El Cerebro)
    # Usamos menos generaciones para la demo rápida (50 en vez de 300)
    optimizer = GeneticOptimizer(planner, pop_size=50, generations=50)
    
    best_angle, best_path, metrics = optimizer.optimize(safe_field)
    
    print("\n--- RESULTADOS ---")
    print(f"Ángulo Óptimo: {best_angle:.3f}°")
    print(f"Distancia de Vuelo (l): {metrics['l']:.2f} m")
    print(f"Área Cubierta (S'): {metrics['s_prime']:.2f} m²")
    print(f"Error de Cobertura (eta): {metrics['eta']:.2f}%")
    print(f"Fitness Final: {metrics['fitness']:.5f}")

    # 5. Visualización
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Dibujar campo original y reducido
    x_orig, y_orig = original_field.exterior.xy
    ax.plot(x_orig, y_orig, 'k-', linewidth=2, label='Límite Real')
    
    x_safe, y_safe = safe_field.exterior.xy
    ax.plot(x_safe, y_safe, 'r--', linewidth=1, label='Margen de Seguridad')
    
    # Dibujar la MEJOR ruta encontrada
    path_x = [p[0] for p in best_path]
    path_y = [p[1] for p in best_path]
    ax.plot(path_x, path_y, 'b.-', markersize=2, label=f'Ruta Óptima ({best_angle:.1f}°)')
    
    plt.title(f"Planificación Genética - Fitness: {metrics['fitness']:.4f}")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()