# examples/demo_visualizer.py
import sys
import os
import matplotlib.pyplot as plt

# --- TRUCO PRO: Añadir la carpeta 'src' al path para poder importar ---
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from geometry import FieldGenerator # Importamos TU librería

def main():
    # 1. Instanciar la lógica
    campo = FieldGenerator.create_l_shape_field()
    
    # 2. Visualizar (Lógica de UI separada de la matemática)
    x, y = campo.exterior.xy
    
    plt.figure(figsize=(6, 6))
    plt.plot(x, y, label="Límite del Campo", color='black', linewidth=2)
    plt.fill(x, y, alpha=0.3, color='green')
    plt.title("Visualización Inicial - AgriSwarm")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()