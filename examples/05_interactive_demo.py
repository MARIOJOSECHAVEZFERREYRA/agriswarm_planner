import sys
import os
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.offsetbox import AnchoredText

# Setup path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from field_editor import InteractiveEditor
from field_io import FieldIO
from margin import MarginReducer
from path_planner import BoustrophedonPlanner
from genetic_optimizer import GeneticOptimizer

# --- CLASE PARA LA VENTANA DE RESULTADOS (DASHBOARD) ---
class ResultDashboard:
    def __init__(self, campo_raw, campo_seguro, best_path, best_angle, metrics):
        self.should_restart = False
        
        # Configurar ventana
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.fig.canvas.manager.set_window_title('AgriSwarm - Resultado de Misión')
        plt.subplots_adjust(bottom=0.15) # Espacio para botones

        # --- DIBUJAR GEOMETRÍA ---
        x_o, y_o = campo_raw.exterior.xy
        self.ax.fill(x_o, y_o, alpha=0.1, fc='gray')
        self.ax.plot(x_o, y_o, 'k-', linewidth=3, label='Límite Catastral')
        
        x_s, y_s = campo_seguro.exterior.xy
        self.ax.plot(x_s, y_s, 'r--', linewidth=1, label='Margen Seguridad')
        
        if best_path:
            px = [p[0] for p in best_path]
            py = [p[1] for p in best_path]
            self.ax.plot(px, py, 'b.-', linewidth=1.5, markersize=3, label=f'Ruta ({best_angle:.1f}°)')
            self.ax.scatter(px[0], py[0], c='green', s=150, zorder=5, label='Inicio')
            self.ax.scatter(px[-1], py[-1], c='red', s=150, marker='X', zorder=5, label='Fin')

        # --- CAJA DE INFORMACIÓN ---
        # Cálculos "humanos"
        area_has = campo_raw.area / 10000.0
        distancia_km = metrics['l'] / 1000.0
        tiempo_estimado_min = (metrics['l'] / 5.0) / 60.0 # Asumiendo 5 m/s
        eficiencia = 100 - metrics['eta']

        info_text = (
            f"📊 REPORTE DE MISIÓN\n"
            f"────────────────────────\n"
            f"🚜 Área:      {area_has:.2f} ha\n"
            f"🧭 Ángulo:    {best_angle:.1f}°\n"
            f"📏 Distancia: {distancia_km:.2f} km\n"
            f"⏱️ Tiempo:    {tiempo_estimado_min:.1f} min\n"
            f"🎯 Eficiencia: {eficiencia:.1f}%"
        )
        
        text_box = AnchoredText(info_text, loc='upper left', frameon=True, prop=dict(size=11))
        text_box.patch.set_boxstyle("round,pad=0.5,rounding_size=0.2")
        text_box.patch.set_facecolor("aliceblue")
        self.ax.add_artist(text_box)

        # Decoración
        self.ax.set_title(f"Planificación Optimizada - Fitness: {metrics['fitness']:.2f}", fontsize=14, fontweight='bold')
        self.ax.legend(loc='upper right')
        self.ax.axis('equal')
        self.ax.grid(True, linestyle='--', alpha=0.6)

        # --- BOTONES DE CONTROL ---
        ax_restart = plt.axes([0.3, 0.02, 0.2, 0.08])
        ax_exit = plt.axes([0.55, 0.02, 0.15, 0.08])

        self.btn_restart = Button(ax_restart, '🔄 Nueva Misión', color='lightblue', hovercolor='skyblue')
        self.btn_exit = Button(ax_exit, '❌ Salir', color='lightcoral', hovercolor='red')

        self.btn_restart.on_clicked(self.restart)
        self.btn_exit.on_clicked(self.exit_app)

    def restart(self, event):
        print("🔄 Reiniciando sistema...")
        self.should_restart = True
        plt.close(self.fig) # Cerrar ventana rompe el bloqueo de show()

    def exit_app(self, event):
        print("👋 Cerrando AgriSwarm...")
        self.should_restart = False
        plt.close(self.fig)

    def show(self):
        plt.show()
        return self.should_restart


# --- LOGICA PRINCIPAL EN BUCLE ---
def main():
    # Rutas
    data_dir = os.path.join(os.path.dirname(__file__), '..', 'data')
    filename = os.path.join(data_dir, 'mi_campo_dibujado.json')

    while True: # BUCLE INFINITO DE LA APLICACIÓN
        plt.close('all') # Limpiar cualquier figura basura
        print("\n🚜 --- AGRISWARM MISSION PLANNER --- 🚜")
        
        # 1. EDITOR
        editor = InteractiveEditor(load_filename=filename)
        campo_raw = editor.show() 

        if campo_raw is None:
            print("❌ Operación cancelada en el editor.")
            break # Salir del programa

        # Guardar automáticamente
        FieldIO.save_field(campo_raw, filename)

        # 2. PROCESAMIENTO
        print("\n⚙️  Procesando geometría...")
        MARGIN_M = 2.0
        try:
            campo_seguro = MarginReducer.shrink(campo_raw, margin_h=MARGIN_M)
        except Exception as e:
            print(f"Error geométrico crítico: {e}")
            continue # Volver a empezar

        print("🧬 Ejecutando Algoritmo Genético (Li et al. 2023)...")
        # Parámetros rápidos para la demo (puedes aumentar generations para más precisión)
        planner = BoustrophedonPlanner(spray_width=5.0)
        optimizer = GeneticOptimizer(planner, pop_size=60, generations=30) 
        
        best_angle, best_path, metrics = optimizer.optimize(campo_seguro)

        # 3. DASHBOARD (Ahora con botones)
        print("\n✅ Mostrando resultados...")
        dashboard = ResultDashboard(campo_raw, campo_seguro, best_path, best_angle, metrics)
        restart = dashboard.show() # Esto bloquea hasta que toques un botón

        if not restart:
            break # Romper el bucle infinito y salir

if __name__ == "__main__":
    main()