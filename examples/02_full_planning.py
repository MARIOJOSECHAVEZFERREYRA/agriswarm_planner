# examples/02_full_planning.py
import sys
import os
import matplotlib.pyplot as plt

# Agregar 'src' al path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

# Importar TUS módulos
from geometry import FieldGenerator
from divider import FieldDivider
from path_planner import CoveragePlanner

def main():
    # 1. Configuración
    NUM_DRONES = 3
    ANCHO_ASPERSION = 8.0 # Metros
    
    # 2. Generar Campo (Usamos la forma de L para probar la optimización)
    campo = FieldGenerator.create_l_shape_field()
    
    # 3. Dividir Tareas
    print(f"Dividiendo campo para {NUM_DRONES} drones...")
    sub_areas = FieldDivider.divide_vertically(campo, NUM_DRONES)
    
    # 4. Planificar Rutas (Aquí ocurre la optimización automática)
    planner = CoveragePlanner(sweep_width=ANCHO_ASPERSION)
    rutas_generadas = []
    
    print("Calculando rutas óptimas...")
    for i, area in enumerate(sub_areas):
        ruta, angulo = planner.generate_optimized_path(area)
        rutas_generadas.append(ruta)
        print(f"  -> Dron {i+1}: Ángulo óptimo encontrado: {angulo}°")

    # 5. Visualización Profesional
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Dibujar borde del campo original
    x, y = campo.exterior.xy
    ax.plot(x, y, 'k-', linewidth=3, label='Límite Campo')
    
    colores = ['#FF5733', '#33FF57', '#3357FF'] # Hex codes se ven mejor
    
    for i, (area, ruta) in enumerate(zip(sub_areas, rutas_generadas)):
        # Fondo de color suave para el área asignada
        x_a, y_a = area.exterior.xy
        ax.fill(x_a, y_a, color=colores[i], alpha=0.2)
        
        # Dibujar la ruta de vuelo
        coord_x = []
        coord_y = []
        for segmento in ruta:
            coord_x.extend([p[0] for p in segmento])
            coord_y.extend([p[1] for p in segmento])
            
        ax.plot(coord_x, coord_y, color=colores[i], marker='.', markersize=4, label=f'Dron {i+1}')

    plt.title("AgriSwarm: Planificación Multi-Dron Optimizada")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()