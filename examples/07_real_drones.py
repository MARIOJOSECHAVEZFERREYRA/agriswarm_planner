import sys
import os
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons
from matplotlib.offsetbox import AnchoredText
from shapely.geometry import Point, LineString
from shapely.ops import substring

# Setup path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from field_editor import InteractiveEditor as BaseEditor
from field_io import FieldIO
from margin import MarginReducer
from path_planner import BoustrophedonPlanner
from genetic_optimizer import GeneticOptimizer
from geo_utils import GeoUtils
# Importamos la estructura y los datos
from drone_db import DroneDB 
import drone_data 

# --- 1. EDITOR EXTENDIDO ---
class MissionConfigurator(BaseEditor):
    def __init__(self, load_filename=None):
        super().__init__(load_filename) 
        self.selected_drone = "DJI Agras T30"
        
        ax_radio = plt.axes([0.75, 0.6, 0.2, 0.25], facecolor='#f0f0f0')
        self.radio = RadioButtons(ax_radio, DroneDB.get_drone_names())
        self.radio.on_clicked(self.set_drone)
        
        self.fig.text(0.75, 0.86, "🚁 SELECCIONAR DRON", fontsize=10, fontweight='bold')
        self.fig.text(0.75, 0.55, "Datos: Especificaciones Técnicas", fontsize=7, style='italic')

    def set_drone(self, label):
        self.selected_drone = label
        print(f"🔹 Dron seleccionado: {label}")

    def show(self):
        plt.show()
        return self.final_polygon, self.selected_drone

# --- 2. DASHBOARD SIN COSTOS (CORREGIDO) ---
class ResultDashboard:
    def __init__(self, campo_raw, campo_seguro, best_path, best_angle, metrics, drone_name):
        self.should_restart = False
        self.best_path = best_path
        
        specs = DroneDB.get_specs(drone_name)
        
        self.fig, self.ax = plt.subplots(figsize=(13, 10))
        self.fig.canvas.manager.set_window_title(f'AgriSwarm - Misión: {drone_name}')
        plt.subplots_adjust(bottom=0.15, right=0.75) 

        # Geometría
        x_o, y_o = campo_raw.exterior.xy
        self.ax.fill(x_o, y_o, alpha=0.2, fc='gray', label='Cultivo')
        self.ax.plot(x_o, y_o, 'k-', linewidth=2)
        
        x_s, y_s = campo_seguro.exterior.xy
        self.ax.plot(x_s, y_s, 'r--', linewidth=1, alpha=0.5)
        
        info_text = "Error en ruta."
        alert_color = "aliceblue"

        if best_path:
            px = [p[0] for p in best_path]
            py = [p[1] for p in best_path]
            self.ax.plot(px, py, 'b.-', linewidth=1, markersize=2, label='Ruta Vuelo')
            
            start_point = Point(px[0], py[0])
            end_point = Point(px[-1], py[-1])
            self.ax.scatter(start_point.x, start_point.y, c='green', s=150, zorder=5, edgecolors='k')
            self.ax.scatter(end_point.x, end_point.y, c='orange', s=150, marker='*', zorder=5, edgecolors='k')

            road_poly = campo_raw.buffer(5.0, join_style=2)
            truck_path, truck_dist = self.calculate_perimeter_path(road_poly.exterior, start_point, end_point)
            if truck_path:
                tx, ty = truck_path.xy
                self.ax.plot(tx, ty, color='orange', linestyle='--', linewidth=2, label='Camioneta')

            # --- CÁLCULOS TÉCNICOS ---
            dist_vuelo_metros = metrics['l']
            dist_vuelo_km = dist_vuelo_metros / 1000.0
            area_ha = campo_raw.area / 10000.0
            
            # Velocidad y Tiempo
            if specs.flight.work_speed_kmh:
                velocidad_kmh = float(specs.flight.work_speed_kmh.value)
            else:
                velocidad_kmh = float(specs.flight.max_speed_kmh.value)
            
            velocidad_ms = velocidad_kmh / 3.6
            tiempo_min = (dist_vuelo_metros / velocidad_ms) / 60.0

            # Batería
            time_key = "hover_loaded" if specs.category == "spray" else "standard"
            rango_max_km = DroneDB.theoretical_range_km(specs, time_key=time_key, use_work_speed=True)
            
            if rango_max_km:
                bateria_usada_pct = (dist_vuelo_km / rango_max_km) * 100
            else:
                bateria_usada_pct = 0.0

            # FUMIGACIÓN
            info_spray = ""
            if specs.category == "spray" and specs.spray:
                DOSIS_L_HA = 20.0 
                tanque_l = float(specs.spray.tank_l.value)
                litros_totales = area_ha * DOSIS_L_HA
                tanques_necesarios = math.ceil(litros_totales / tanque_l)
                
                info_spray = (
                    f"──────────────────────\n"
                    f"💧 FUMIGACIÓN (20 L/ha):\n"
                    f"Vol. Total:  {litros_totales:.1f} L\n"
                    f"Tanques:     {tanques_necesarios} recargas\n"
                )
                if tanques_necesarios > 1:
                    alert_color = "#fff5cc"

            status_bateria = "OK"
            if bateria_usada_pct > 90:
                status_bateria = "CRÍTICO ⚠️"
                alert_color = "#ffcccc"
            if bateria_usada_pct > 100:
                status_bateria = "AGOTADA ❌"
                alert_color = "#ff9999"

            # --- TEXTO DE REPORTE (SIN PRECIOS) ---
            info_text = (
                f"📋 REPORTE TÉCNICO\n"
                f"──────────────────────\n"
                f"🚁 Nave:    {drone_name}\n"
                f"📏 Distancia: {dist_vuelo_metros:.0f} m\n"
                f"   ({dist_vuelo_km:.2f} km)\n"
                f"⏱️ Tiempo:    {tiempo_min:.1f} min\n"
                f"🔋 Batería:   {bateria_usada_pct:.1f}% [{status_bateria}]\n"
                f"{info_spray}"
                f"──────────────────────\n"
                f"🚛 Logística:\n"
                f"Mover camión: {truck_dist:.0f} m\n"
                f"Ahorro vuelo: {(truck_dist/(dist_vuelo_metros+truck_dist))*100:.1f}%"
            )

        self.fig.text(0.77, 0.5, info_text, fontsize=10, family='monospace',
                      bbox=dict(boxstyle="round,pad=0.5", fc=alert_color, ec="gray"))

        ax_restart = plt.axes([0.15, 0.02, 0.2, 0.08])
        ax_export = plt.axes([0.40, 0.02, 0.2, 0.08])
        ax_exit = plt.axes([0.65, 0.02, 0.15, 0.08])

        self.btn_restart = Button(ax_restart, '🔄 Nueva Misión', color='lightblue')
        self.btn_export = Button(ax_export, '💾 Exportar GPS', color='lightgreen')
        self.btn_exit = Button(ax_exit, '❌ Salir', color='lightcoral')

        self.btn_restart.on_clicked(self.restart)
        self.btn_export.on_clicked(self.export_mission)
        self.btn_exit.on_clicked(self.exit_app)
        
        self.ax.set_title(f"Planificación {drone_name} - Fitness: {metrics['fitness']:.2f}")
        self.ax.axis('equal')
        self.ax.grid(True, linestyle=':', alpha=0.6)

    def calculate_perimeter_path(self, ring, p_start, p_end):
        d_start, d_end = ring.project(p_start), ring.project(p_end)
        if d_start == d_end: return None, 0
        p1, p2 = min(d_start, d_end), max(d_start, d_end)
        dist_A = p2 - p1
        dist_B = ring.length - dist_A
        if dist_A < dist_B:
            return substring(ring, p1, p2), dist_A
        else:
            part1 = substring(ring, p2, ring.length)
            part2 = substring(ring, 0, p1)
            return LineString(list(part1.coords) + list(part2.coords)[1:]), dist_B

    def export_mission(self, event):
        if not self.best_path: return
        GeoUtils.export_qgc_mission(self.best_path, "mision_real.plan")
        print("✅ Misión exportada.")

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
        print("\n🚜 --- AGRISWARM MISSION PLANNER PRO --- 🚜")
        
        # 1. EDITOR
        editor = MissionConfigurator(load_filename=filename)
        result = editor.show()
        
        if result is None: break
        campo_raw, drone_name = result
        if campo_raw is None: break

        FieldIO.save_field(campo_raw, filename)
        print(f"\n⚙️  Planificando para {drone_name}...")
        
        # Obtener especificaciones
        specs = DroneDB.get_specs(drone_name)
        
        # --- AQUÍ ESTÁ EL CAMBIO LIMPIO ---
        # Delegamos el cálculo complejo a la clase DroneDB
        margen_calculado = DroneDB.calculate_safety_margin_m(specs, buffer_gps=0.5)
        
        print(f"   -> Margen de seguridad inteligente (h): {margen_calculado} m")
        
        try:
            campo_seguro = MarginReducer.shrink(campo_raw, margin_h=margen_calculado)
        except Exception as e:
            print(f"⚠️ Error: El campo es muy pequeño para este margen ({margen_calculado}m).")
            continue

        # Configurar Swath
        real_swath = 5.0
        if specs.spray and specs.spray.swath_m:
            min_s = float(specs.spray.swath_m[0].value)
            max_s = float(specs.spray.swath_m[1].value)
            real_swath = (min_s + max_s) / 2.0

        planner = BoustrophedonPlanner(spray_width=real_swath)
        optimizer = GeneticOptimizer(planner, pop_size=60, generations=40)
        best_angle, best_path, metrics = optimizer.optimize(campo_seguro)

        dashboard = ResultDashboard(campo_raw, campo_seguro, best_path, best_angle, metrics, drone_name)
        if not dashboard.show(): break

if __name__ == "__main__":
    main()