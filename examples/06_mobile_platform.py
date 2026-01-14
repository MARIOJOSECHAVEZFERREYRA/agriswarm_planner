import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.offsetbox import AnchoredText
from shapely.geometry import Point, LineString
from shapely.ops import substring

# Setup path para importar src
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from field_editor import InteractiveEditor
from field_io import FieldIO
from margin import MarginReducer
from path_planner import BoustrophedonPlanner
from genetic_optimizer import GeneticOptimizer
from geo_utils import GeoUtils

# --- CLASE PARA LA VENTANA DE RESULTADOS (DASHBOARD) ---
class ResultDashboard:
    def __init__(self, campo_raw, campo_seguro, best_path, best_angle, metrics):
        self.should_restart = False
        self.best_path = best_path 
        
        # Configurar ventana
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.fig.canvas.manager.set_window_title('AgriSwarm - Logística Terrestre Inteligente')
        plt.subplots_adjust(bottom=0.15)

        # --- DIBUJAR GEOMETRÍA ---
        # 1. Campo Real (Gris)
        x_o, y_o = campo_raw.exterior.xy
        self.ax.fill(x_o, y_o, alpha=0.2, fc='gray', label='Cultivo')
        self.ax.plot(x_o, y_o, 'k-', linewidth=2, label='Límite Catastral')
        
        # 2. Margen de Seguridad Dron (Rojo interno)
        x_s, y_s = campo_seguro.exterior.xy
        self.ax.plot(x_s, y_s, 'r--', linewidth=1, alpha=0.5, label='Límite Vuelo')
        
        info_text = "No se generó ruta."
        
        if best_path:
            # --- DATOS DE RUTA DRON ---
            px = [p[0] for p in best_path]
            py = [p[1] for p in best_path]
            self.ax.plot(px, py, 'b.-', linewidth=1.5, markersize=3, label=f'Ruta Dron')
            
            start_point = Point(px[0], py[0])
            end_point = Point(px[-1], py[-1])

            # --- LÓGICA DE CAMIONETA (NUEVO) ---
            # Crear "Carretera Perimetral": Inflamos el campo hacia afuera (Buffer positivo)
            # Usamos el mismo margen que el dron (ej. 2m) o un poco más para la camioneta (ej. 5m)
            TRUCK_OFFSET = 5.0 
            road_poly = campo_raw.buffer(TRUCK_OFFSET, join_style=2) # join_style=2 es esquinas rectas (mitre)
            road_ring = road_poly.exterior
            
            # Calcular la ruta óptima por el borde (Izquierda vs Derecha)
            truck_path_geom, truck_dist = self.calculate_perimeter_path(road_ring, start_point, end_point)
            
            # Dibujar la ruta de la camioneta
            if truck_path_geom:
                tx, ty = truck_path_geom.xy
                self.ax.plot(tx, ty, color='orange', linestyle='-', linewidth=3, alpha=0.8, label='Ruta Camioneta')
                # Flechas de dirección para la camioneta
                mid_idx = len(tx) // 2
                self.ax.annotate("", xy=(tx[mid_idx], ty[mid_idx]), xytext=(tx[mid_idx-1], ty[mid_idx-1]),
                                 arrowprops=dict(arrowstyle="->", color='orange', lw=3))

            # Puntos de Inicio y Fin
            self.ax.scatter(start_point.x, start_point.y, c='green', s=180, zorder=10, edgecolors='k', label='Inicio')
            self.ax.scatter(end_point.x, end_point.y, c='orange', s=180, marker='*', zorder=10, edgecolors='k', label='Recogida')

            # --- MÉTRICAS FINALES ---
            area_has = campo_raw.area / 10000.0
            distancia_vuelo = metrics['l']
            distancia_vuelo_km = distancia_vuelo / 1000.0
            tiempo_vuelo_min = (distancia_vuelo / 5.0) / 60.0
            
            # Comparativa: ¿Cuánto ahorramos vs. Dron volviendo al inicio?
            # Si el dron tuviera que volver volando (RTL), volaría en línea recta.
            distancia_retorno_aereo = start_point.distance(end_point)
            distancia_total_sin_camioneta = distancia_vuelo + distancia_retorno_aereo
            
            # Ahorro de batería del dron (por no volar la vuelta)
            ahorro_bateria = (distancia_retorno_aereo / distancia_total_sin_camioneta) * 100

            info_text = (
                f"📊 LOGÍSTICA DE MISIÓN\n"
                f"────────────────────────\n"
                f"🚜 Área Cultivo:    {area_has:.2f} ha\n"
                f"🛸 Vuelo Dron:      {distancia_vuelo_km:.2f} km\n"
                f"⏱️ Tiempo Vuelo:    {tiempo_vuelo_min:.1f} min\n"
                f"────────────────────────\n"
                f"🚛 Ruta Camioneta:  {truck_dist:.0f} m\n"
                f"⚡ AHORRO DRON:     {ahorro_bateria:.1f}%\n"
                f"(Al evitar el retorno aéreo)"
            )

        # --- CAJA DE INFORMACIÓN ---
        text_box = AnchoredText(info_text, loc='upper left', frameon=True, prop=dict(size=10, family='monospace'))
        text_box.patch.set_boxstyle("round,pad=0.5,rounding_size=0.2")
        text_box.patch.set_facecolor("aliceblue")
        self.ax.add_artist(text_box)

        self.ax.set_title(f"Planificación Cooperativa (Dron + Vehículo) - Fitness: {metrics['fitness']:.2f}", fontsize=13, fontweight='bold')
        self.ax.legend(loc='upper right', fontsize=9)
        self.ax.axis('equal')
        self.ax.grid(True, linestyle=':', alpha=0.6)

        # --- BOTONES ---
        ax_restart = plt.axes([0.15, 0.02, 0.2, 0.08])
        ax_export = plt.axes([0.40, 0.02, 0.2, 0.08])
        ax_exit = plt.axes([0.65, 0.02, 0.15, 0.08])

        self.btn_restart = Button(ax_restart, '🔄 Nueva Misión', color='lightblue', hovercolor='skyblue')
        self.btn_export = Button(ax_export, '💾 Exportar GPS', color='lightgreen', hovercolor='lime')
        self.btn_exit = Button(ax_exit, '❌ Salir', color='lightcoral', hovercolor='red')

        self.btn_restart.on_clicked(self.restart)
        self.btn_export.on_clicked(self.export_mission)
        self.btn_exit.on_clicked(self.exit_app)

    def calculate_perimeter_path(self, ring, p_start, p_end):
        """
        Calcula la ruta más corta a lo largo del anillo (borde) entre dos puntos.
        """
        # 1. Proyectar puntos del dron (dentro) al camino de la camioneta (fuera)
        d_start = ring.project(p_start)
        d_end = ring.project(p_end)
        total_length = ring.length

        if d_start == d_end: return None, 0

        # Asegurar orden menor -> mayor para facilitar lógica
        p1, p2 = min(d_start, d_end), max(d_start, d_end)

        # 2. Calcular las dos opciones: Directa o Vuelta Completa
        # Opción A: Ir de p1 a p2 directamente
        dist_A = p2 - p1
        # Opción B: Ir de p1 hacia atrás pasando por el 0 (vuelta al mundo)
        dist_B = total_length - dist_A

        # 3. Construir la geometría del camino ganador
        if dist_A < dist_B:
            # Camino simple a lo largo de la línea
            path = substring(ring, p1, p2)
            final_dist = dist_A
        else:
            # Camino complejo (cruza el punto de inicio/cierre del polígono)
            # Unimos: [p2 -> Final] + [Inicio -> p1]
            part1 = substring(ring, p2, total_length)
            part2 = substring(ring, 0, p1)
            # Unir coordenadas
            coords = list(part1.coords) + list(part2.coords)[1:] # Evitar duplicar punto de unión
            path = LineString(coords)
            final_dist = dist_B
            
        return path, final_dist

    def export_mission(self, event):
        if not self.best_path: return
        filename = "mision_agriswarm_logistica.plan"
        GeoUtils.export_qgc_mission(self.best_path, filename, home_lat=-17.3935, home_lon=-63.2622)
        print(f"✅ Archivo '{filename}' generado.")
        self.btn_export.color = 'gold'
        self.btn_export.label.set_text("¡Guardado!")
        self.fig.canvas.draw()

    def restart(self, event):
        self.should_restart = True
        plt.close(self.fig)

    def exit_app(self, event):
        self.should_restart = False
        plt.close(self.fig)

    def show(self):
        plt.show()
        return self.should_restart

# --- MAIN LOOP ---
def main():
    data_dir = os.path.join(os.path.dirname(__file__), '..', 'data')
    filename = os.path.join(data_dir, 'mi_campo_dibujado.json')

    while True:
        plt.close('all')
        print("\n🚜 --- AGRISWARM MISSION PLANNER --- 🚜")
        
        editor = InteractiveEditor(load_filename=filename)
        campo_raw = editor.show()
        if campo_raw is None: break
        FieldIO.save_field(campo_raw, filename)

        print("\n⚙️  Procesando geometría...")
        try:
            campo_seguro = MarginReducer.shrink(campo_raw, margin_h=2.0)
        except: continue

        print("🧬 Optimizando ruta...")
        planner = BoustrophedonPlanner(spray_width=5.0)
        optimizer = GeneticOptimizer(planner, pop_size=60, generations=40)
        best_angle, best_path, metrics = optimizer.optimize(campo_seguro)

        dashboard = ResultDashboard(campo_raw, campo_seguro, best_path, best_angle, metrics)
        if not dashboard.show(): break

if __name__ == "__main__":
    main()