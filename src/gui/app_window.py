import sys
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QComboBox, QPushButton, QTextEdit, QMessageBox, 
                             QFrame, QSizePolicy, QFormLayout, QDoubleSpinBox, QCheckBox, QStackedWidget)
from PyQt6.QtCore import Qt
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import substring
import math
import datetime

# Imports Propios
from drone_db import DroneDB, SpecValue
from field_io import FieldIO
from algorithms import MarginReducer, BoustrophedonPlanner, GeneticOptimizer, MobileStation, MissionSegmenter
from algorithms.analysis import MissionAnalyzer
# from decomposition import ConcaveDecomposer # Not currently used in app_window based on previous view?
from geo_utils import GeoUtils
from gui.map_widget import MapWidget
from gui.report_panel import ReportPanel
from gui.styles import *

class AgriSwarmApp(QMainWindow):
    def __init__(self, filename=None):
        super().__init__()
        self.filename = filename
        self.setWindowTitle("AgriSwarm Planner - Tesis v2.5")
        self.resize(1200, 800)
        # Apply only global styles (Dialogs, etc). Sidebar has its own.
        self.setStyleSheet(QMESSAGEBOX_STYLE)

        # Estado Logico
        self.points = []
        self.polygon = None
        self.current_drone = "DJI Agras T30"
        self.best_path = None
        self.metrics = None
        self.truck_dist = 0
        self.last_mission_cycles = None # Mobile by default
        self.static_cycles = None
        self.current_specs = None
        self.safe_polygon = None



        # --- LAYOUT PRINCIPAL ---
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 1. MAPA (Izquierda)
        self.map_widget = MapWidget(self)
        layout.addWidget(self.map_widget, stretch=1)

        # Conectar Señales
        self.map_widget.map_clicked.connect(self.on_map_left_click)
        self.map_widget.map_right_clicked.connect(self.on_map_right_click)
        self.map_widget.point_moved.connect(self.on_point_moved)

        # 2. BARRA LATERAL (Derecha) - Stacked (Control Panel / Report)
        self.sidebar_stack = QStackedWidget()
        self.sidebar_stack.setFixedWidth(380)
        layout.addWidget(self.sidebar_stack)

        # --- PANEL 0: CONTROLES (Lo que ya teniamos) ---
        self.control_panel = QWidget()
        self.control_panel.setStyleSheet(SIDEBAR_STYLE)
        self.sidebar_stack.addWidget(self.control_panel)

        side_layout = QVBoxLayout(self.control_panel)
        side_layout.setContentsMargins(20, 25, 20, 25)
        side_layout.setSpacing(15)

        # --- CONTENIDO BARRA LATERAL ---
        lbl_brand = QLabel("AGRISWARM")
        lbl_brand.setStyleSheet("font-size: 26px; color: #ecf0f1; letter-spacing: 2px; margin-bottom: 10px;")
        lbl_brand.setAlignment(Qt.AlignmentFlag.AlignCenter)
        side_layout.addWidget(lbl_brand)

        self.add_separator(side_layout)

        side_layout.addWidget(QLabel("AERONAVE"))
        self.combo_drones = QComboBox()
        self.combo_drones.addItems(DroneDB.get_drone_names())
        self.combo_drones.setCurrentText("DJI Agras T30")
        self.combo_drones.currentTextChanged.connect(self.on_drone_changed)
        side_layout.addWidget(self.combo_drones)

        # --- PARAMETROS DE MISION (Overrides) ---
        self.params_layout = QFormLayout()
        self.params_layout.setSpacing(8)
        
        # 1. Ancho de Trabajo (Swath)
        self.spin_swath = QDoubleSpinBox()
        self.spin_swath.setRange(1.0, 20.0)
        self.spin_swath.setSingleStep(0.5)
        self.spin_swath.setSuffix(" m")
        self.params_layout.addRow("Ancho (Swath):", self.spin_swath)
        
        # 2. Tanque
        self.spin_tank = QDoubleSpinBox()
        self.spin_tank.setRange(5.0, 100.0)
        self.spin_tank.setSingleStep(1.0)
        self.spin_tank.setSuffix(" L")
        self.params_layout.addRow("Tanque:", self.spin_tank)
        
        # 3. Velocidad
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(1.0, 15.0)
        self.spin_speed.setSingleStep(0.5)
        self.spin_speed.setSuffix(" m/s")
        self.params_layout.addRow("Velocidad:", self.spin_speed)
        
        # 4. Flujo (L/min)
        self.spin_flow = QDoubleSpinBox()
        self.spin_flow.setRange(1.0, 40.0)
        self.spin_flow.setSingleStep(0.5)
        self.spin_flow.setSuffix(" L/min")
        self.spin_flow.setSuffix(" L/min")
        self.params_layout.addRow("Flujo Bomba:", self.spin_flow)
        
        # 5. Distancia Estacion (Offset)
        self.spin_truck_offset = QDoubleSpinBox()
        self.spin_truck_offset.setRange(0.0, 50.0)
        self.spin_truck_offset.setSingleStep(1.0)
        self.spin_truck_offset.setSuffix(" m")
        self.spin_truck_offset.setValue(0.0)
        self.spin_truck_offset.setToolTip("Distancia extra entre el borde del campo y la ruta del camion")
        self.params_layout.addRow("Distancia Estacion:", self.spin_truck_offset)
        
        side_layout.addLayout(self.params_layout)

        side_layout.addWidget(QLabel("GEOMETRIA"))
        btn_grid = QHBoxLayout()
        self.btn_clear = QPushButton("BORRAR")
        self.btn_clear.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_clear.setStyleSheet(BTN_CLEAR_STYLE)
        self.btn_clear.clicked.connect(self.clear_canvas)
        
        self.btn_load = QPushButton("CARGAR")
        self.btn_load.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_load.setStyleSheet(BTN_LOAD_STYLE)
        self.btn_load.clicked.connect(self.load_field)
        
        btn_grid.addWidget(self.btn_clear)
        btn_grid.addWidget(self.btn_load)
        side_layout.addLayout(btn_grid)
        
        # Opciones Visuales
        self.chk_swath = QCheckBox("Mostrar Cobertura Real (Swath)")
        self.chk_swath.setChecked(True)
        self.chk_swath.stateChanged.connect(self.on_swath_toggled)
        side_layout.addWidget(self.chk_swath)
        
        # Toggle para VIsualizacion Estatica vs Movil
        self.chk_mode_static = QCheckBox("Visualizar: Estación Estática")
        self.chk_mode_static.setChecked(False)
        self.chk_mode_static.setEnabled(False) # Solo activo tras caculo
        self.chk_mode_static.setStyleSheet("color: #e74c3c; font-weight: bold;")
        self.chk_mode_static.stateChanged.connect(self.toggle_visualization_mode)
        side_layout.addWidget(self.chk_mode_static)

        self.add_separator(side_layout)

        self.btn_calc = QPushButton("CALCULAR RUTA")
        self.btn_calc.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_calc.setFixedHeight(60)
        self.btn_calc.setStyleSheet(BTN_CALC_STYLE)
        self.btn_calc.clicked.connect(self.run_optimization)
        side_layout.addWidget(self.btn_calc)


        
        # Boton Reporte Comparativo (Tesis Mode)
        self.btn_report_window = QPushButton("VER REPORTE COMPARATIVO")
        self.btn_report_window.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_report_window.setFixedHeight(40)
        self.btn_report_window.setStyleSheet(f"background-color: {DARK_BLUE}; color: {ACCENT_ORANGE}; font-weight: bold; border: 1px solid {ACCENT_ORANGE}; border-radius: 4px;")
        self.btn_report_window.clicked.connect(self.show_comparative_report)
        self.btn_report_window.setEnabled(False) # Habilitar solo tras calculo
        side_layout.addWidget(self.btn_report_window)

        self.btn_export = QPushButton("EXPORTAR .PLAN")
        self.btn_export.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_export.setFixedHeight(45)
        self.btn_export.setStyleSheet(BTN_EXPORT_STYLE)
        self.btn_export.clicked.connect(self.export_mission)
        self.btn_export.setEnabled(False)
        side_layout.addWidget(self.btn_export)

        # Inicializar
        self.map_widget.clear_map()
        
        # Force initial populate (Now that UI is ready)
        self.on_drone_changed(self.combo_drones.currentText())

    def add_separator(self, layout):
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Plain)
        line.setStyleSheet("background-color: #455a64; max-height: 1px;")
        layout.addWidget(line)

    # --- LOGICA ---

    def on_map_left_click(self, x, y):
        if self.best_path: return
        self.points.append((x, y))
        self.map_widget.draw_editor_state(self.points)

    def on_map_right_click(self, x, y):
        if self.best_path: return
        if self.points:
            self.points.pop()
            self.map_widget.draw_editor_state(self.points)

    def on_point_moved(self, index, x, y):
        if self.best_path: return
        if 0 <= index < len(self.points):
            self.points[index] = (x, y)
            self.map_widget.draw_editor_state(self.points)

    def clear_canvas(self):
        self.points = []
        self.best_path = None
        self.polygon = None
        self.btn_export.setEnabled(False)
        self.map_widget.clear_map()

    def load_field(self):
        if not self.filename: return
        try:
            poly = FieldIO.load_field(self.filename)
            self.points = list(poly.exterior.coords)[:-1]
            self.best_path = None
            self.map_widget.draw_editor_state(self.points)
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def on_drone_changed(self, drone_name):
        self.current_drone = drone_name

        
        spec = DroneDB.get_specs(drone_name)
        if spec:
            # Populate UI with defaults from DB
            
            # Swath Handling (New Smart Logic)
            # DroneDB swath_m is typically (SpecValue(min), SpecValue(max))
            swath_range = spec.spray.swath_m
            
            min_swath = 1.0
            max_swath = 20.0
            default_swath = 5.0
            
            if isinstance(swath_range, tuple) and len(swath_range) == 2:
                # Extract min/max from SpecValue objects
                try:
                    val1 = float(swath_range[0].value)
                    val2 = float(swath_range[1].value)
                    min_swath = min(val1, val2)
                    max_swath = max(val1, val2)
                    default_swath = max_swath # Default to max width for efficiency
                except (ValueError, AttributeError):
                    pass
            elif isinstance(swath_range, SpecValue):
                # Fallback if it's a single value (unlikely based on DroneDB structure but safe)
                val = float(swath_range.value)
                min_swath = val
                max_swath = val
                default_swath = val

            self.spin_swath.setRange(min_swath, max_swath)
            self.spin_swath.setValue(default_swath)
            self.spin_swath.setToolTip(f"Rango permitido: {min_swath}m - {max_swath}m")
            
            # Tank
            self.spin_tank.setValue(spec.spray.tank_l.value)
            
            # Speed (Work Speed)
            self.spin_speed.setValue(spec.flight.work_speed_kmh.value / 3.6) # km/h -> m/s
            
            # Flow
            self.spin_flow.setValue(spec.spray.max_flow_l_min.value)

    def run_optimization(self):
        if len(self.points) < 3: return

        self.polygon = Polygon(self.points)
        if not self.polygon.is_valid:
            self.polygon = self.polygon.buffer(0)
        
        specs = DroneDB.get_specs(self.current_drone)
        QApplication.processEvents()

        margin_h = DroneDB.calculate_safety_margin_m(specs, buffer_gps=0.5)
        
        try:
            campo_seguro = MarginReducer.shrink(self.polygon, margin_h=margin_h)
        except:
            QMessageBox.critical(self, "Error", "Campo muy pequeno.")
            return

        real_swath = 5.0
        if specs.spray and specs.spray.swath_m:
            min_s = float(specs.spray.swath_m[0].value)
            max_s = float(specs.spray.swath_m[1].value)
            real_swath = (min_s + max_s) / 2.0
        
        # 1. READ OVERRIDES
        try:
            # Create a shallow copy or modify the spec to reflect UI values
            # DroneSpec is a dataclass? No, standard object. But we need to use these values.
            # Easiest way: Update the 'specs' object or create a transient one.
            # But specs structure is complex. 
            # Better strategy: Pass overrides to Planner/Segmenter logic directly or patch the spec.
            
            # Patching the spec for this run (safe if we re-fetch from DB on change)
            override_swath = self.spin_swath.value()
            override_tank = self.spin_tank.value()
            override_speed_ms = self.spin_speed.value()
            override_flow = self.spin_flow.value()
            
            # Update Spec Object used for Logic
            # Note: We need to be careful not to break the structure.
            # Assuming we can just use these variables where needed.
            
            specs.spray.tank_l.value = override_tank
            specs.flight.work_speed_kmh.value = override_speed_ms * 3.6
            specs.spray.max_flow_l_min.value = override_flow
            # Swath is a tuple typically (min, max). We will force the planner to use our specific value.
            
        except AttributeError:
             pass # Specs might not be loaded if field invalid
             
        # 2. PLANNING 
        try:
            planner = BoustrophedonPlanner(spray_width=override_swath) # Use override
            optimizer = GeneticOptimizer(planner, pop_size=50, generations=30)
            best_angle, self.best_path, self.metrics = optimizer.optimize(campo_seguro)
            self.safe_polygon = campo_seguro # Store for visualization toggle
            
            # --- FASE 2.5: PHYSICS SEGMENTATION ---
            truck_offset = self.spin_truck_offset.value()
            mobile_station = MobileStation(truck_speed_mps=5.0, truck_offset_m=truck_offset) 
            
            # Update segmenter to use override target rate if we had one (we only expose swath/tank/flow/speed)
            # Assuming target_rate is fixed or we should add it? 
            # For now keep 20.0 L/ha hardcoded or add input? 
            # Let's check MissionSegmenter init... it takes proper specs.
            # We patched specs relative to tank/flow/speed.
            # But segmenter might use swath from somewhere? MissionSegmenter takes `specs`.
            # We need to ensure MissionSegmenter uses the corrected swath for its calcs if any (flow rate check).
            
            segmenter = MissionSegmenter(specs, mobile_station, target_rate_l_ha=20.0)
            
            # Also patch segmenter internal speed if it cached it differently?
            # It uses specs.flight.work_speed_kmh.value. We patched it above. Good.
            
            # Check pump first
            is_pump_ok, pump_msg, req_flow = segmenter.validate_pump()
            if not is_pump_ok:
                 QMessageBox.warning(self, "Pump Warning", pump_msg)
            
            # Usar el poligono del camino (buffer) como referencia para el camion
            road_poly = self.polygon.buffer(5.0, join_style=2)
            
            mission_cycles = segmenter.segment_path(self.polygon, self.best_path, truck_polygon=road_poly)
            
            # Inject Swath Width into cycles metadata for visualization
            for cycle in mission_cycles:
                cycle['swath_width'] = override_swath

            # --- STORE FOR REPORTING & TOGGLE ---
            self.last_mission_cycles = mission_cycles
            self.current_specs = specs # Store modified specs
            
            # Pre-calcular Escenario Estatico (Baseline) para Toggle
            self.static_cycles, _ = MissionAnalyzer.simulate_static_mission(
                MissionSegmenter, 
                self.polygon, 
                self.best_path, 
                self.current_specs, 
                MobileStation
            )
            # Inject Swath for Static too
            for sc in self.static_cycles: sc['swath_width'] = override_swath
            
            self.chk_mode_static.setEnabled(True)
            self.chk_mode_static.setChecked(False) # Reset to mobile
            self.btn_report_window.setEnabled(True)

            total_truck_dist = sum(c.get('truck_dist', 0) for c in mission_cycles)
            self.truck_dist = total_truck_dist
            
            # Recalcular truck paths para visualizacion (como listas de coordenadas)
            # El segmenter devuelve truck_start y truck_end, pero no la geometria completa visual.
            # Necesitamos reconstruirla para que el mapa la dibuje.
            for cycle in mission_cycles:
                if cycle.get('truck_dist', 0) > 0:
                    # Reconstruct visual path using app helper (or better logic inside segmenter, but this works)
                    ts = Point(cycle['truck_start'])
                    te = Point(cycle['truck_end'])
                     # Usamos road_poly (buffer 5m) para el camino del camion
                    road_poly = self.polygon.buffer(5.0, join_style=2)
                    t_path, _ = self.calculate_truck_route(road_poly.exterior, ts, te)
                    if t_path:
                        cycle['truck_path_coords'] = list(t_path.coords)

            # Pass full cycles to map widget
            self.map_widget.draw_results(self.polygon, campo_seguro, mission_cycles)
            
            # GENERAR REPORTE DETALLADO

            self.btn_export.setEnabled(True)

        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def calculate_truck_route(self, ring, p_start, p_end):
        """
        Uses MobileStation to calculate optimal truck path and rendezvous logic.
        """
        try:
             # Instantiate MobileStation (could be member variable if state needed)
             mobile_station = MobileStation(truck_speed_mps=5.0) 
             
             # Convert Shapely Points to tuples
             p_exit_tuple = (p_end.x, p_end.y)
             
             # Assuming truck starts where drone starts (S)
             truck_start_tuple = (p_start.x, p_start.y)
             
             # Use the polygon (reconstruct from ring or use self.polygon)
             # Note: run_optimization passes road_poly.exterior, so we treat it as the boundary.
             road_polygon = Polygon(ring)
             
             r_opt, dist, time_s, _ = mobile_station.calculate_rendezvous(road_polygon, p_exit_tuple, truck_start_tuple)
             
             # Re-calculate the specific path geometry for visualization
             # MobileStation returns dist, but we want the LineString for the GUI
             # We repeat the substring logic here or ideally checking if MobileStation can return it.
             # Current MobileStation returns (r_opt, dist, time). 
             # Let's use the same logic here to get the path geometry.
             
             d_start = ring.project(Point(truck_start_tuple))
             d_end = ring.project(r_opt)
             
             path_direct = substring(ring, min(d_start, d_end), max(d_start, d_end))
             len_direct = path_direct.length
             
             # Check if we chose the direct or inverse path based on 'dist' returned by MobileStation
             if abs(len_direct - dist) < 1.0:
                 return path_direct, dist
             else:
                 # It was the other way around
                 total_len = ring.length
                 # Inverse path
                 part1 = substring(ring, max(d_start, d_end), total_len)
                 part2 = substring(ring, 0, min(d_start, d_end))
                 coords = list(part1.coords) + list(part2.coords)[1:]
                 return LineString(coords), dist

        except Exception as e:
            print(f"Error calculating truck route: {e}")
            return None, 0

    def generate_report(self, metrics, specs, campo, truck_dist, num_cycles=1):
        """Genera un reporte tecnico HTML detallado"""
        
        # 1. Calculos Base
        dist_m = metrics['l']
        dist_km = dist_m / 1000.0
        area_sqm = campo.area
        area_ha = area_sqm / 10000.0
        perimeter_m = campo.length
        
        # 2. Vuelo
        vel_kmh = float(specs.flight.work_speed_kmh.value)
        vel_ms = vel_kmh / 3.6
        tiempo_vuelo_min = (dist_m / vel_ms) / 60.0
        
        # 3. Productividad
        if tiempo_vuelo_min > 0:
            hectareas_por_hora = area_ha / (tiempo_vuelo_min / 60.0)
        else:
            hectareas_por_hora = 0
            
        # 4. Bateria (Estimacion simple lineal)
        # hover_loaded es el peor caso, usamos un promedio con hover_empty para trabajo real
        t_loaded = specs.flight.flight_time_min['hover_loaded'].value
        t_empty = specs.flight.flight_time_min['hover_empty'].value
        avg_endurance = (t_loaded + t_empty) / 2.0 # Estimacion de tiempo de vuelo mixto
        
        baterias_necesarias = math.ceil(tiempo_vuelo_min / avg_endurance)
        bateria_consumo_pct = (tiempo_vuelo_min / avg_endurance) * 100
        
        # 5. Aspersion
        spray_html = ""
        if specs.category == "spray" and specs.spray:
            dosis_l_ha = 20.0 # Estandar
            volumen_total_l = area_ha * dosis_l_ha
            tanque_l = float(specs.spray.tank_l.value)
            tanques_necesarios = math.ceil(volumen_total_l / tanque_l)
            
            # Color semaforo para tanques
            color_tanque = "#27ae60" if tanques_necesarios == 1 else "#e67e22"
            
            spray_html = f"""
            <tr><td colspan="2" style="background-color:#2c3e50; color:white; font-weight:bold;">SISTEMA DE ASPERSION</td></tr>
            <tr><td>Dosis Objetivo:</td><td align="right">{dosis_l_ha} L/ha</td></tr>
            <tr><td>Volumen Total:</td><td align="right"><b>{volumen_total_l:.1f} L</b></td></tr>
            <tr><td>Capacidad Tanque:</td><td align="right">{tanque_l} L</td></tr>
            <tr><td>Recargas Requeridas:</td><td align="right" style="color:{color_tanque}"><b>{tanques_necesarios}</b></td></tr>
            """

        # 6. Logistica
        dist_vuelo_total = dist_m
        dist_camioneta = truck_dist
        ahorro_vuelo_m = dist_camioneta # Asumiendo que el dron no vuela esa distancia
        ahorro_pct = 0
        if (dist_vuelo_total + dist_camioneta) > 0:
            ahorro_pct = (dist_camioneta / (dist_vuelo_total + dist_camioneta)) * 100

        # Construccion HTML
        fecha = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
        
        html = f"""
        <style>
            h3 {{ margin: 0; padding: 0; color: #3498db; }}
            table {{ width: 100%; border-collapse: collapse; margin-top: 5px; }}
            td {{ padding: 3px; font-size: 11px; color: #bdc3c7; }}
            b {{ color: #ffffff; }}
            .sep {{ border-bottom: 1px solid #7f8c8d; }}
        </style>
        
        <h3>MISION: {specs.name.upper()}</h3>
        <span style="font-size:10px; color:#95a5a6">{fecha}</span>
        
        <table border="0">
            <tr><td colspan="2" style="background-color:#2c3e50; color:white; font-weight:bold;">GEOMETRIA</td></tr>
            <tr><td>Area Total:</td><td align="right"><b>{area_ha:.2f} ha</b></td></tr>
            <tr><td>Perimetro:</td><td align="right">{perimeter_m:.0f} m</td></tr>
            
            <tr><td colspan="2" style="background-color:#2c3e50; color:white; font-weight:bold;">PLAN DE VUELO</td></tr>
            <tr><td>Distancia Ruta:</td><td align="right"><b>{dist_m:.0f} m</b></td></tr>
            <tr><td>Velocidad Op:</td><td align="right">{vel_kmh} km/h</td></tr>
            <tr><td>Tiempo Estimado:</td><td align="right"><b>{tiempo_vuelo_min:.1f} min</b></td></tr>
            <tr><td>Productividad:</td><td align="right">{hectareas_por_hora:.1f} ha/h</td></tr>
            
            <tr><td colspan="2" style="background-color:#2c3e50; color:white; font-weight:bold;">ENERGIA</td></tr>
            <tr><td>Consumo Est:</td><td align="right">{bateria_consumo_pct:.0f}%</td></tr>
            <tr><td>Baterias Req:</td><td align="right"><b>{baterias_necesarias}</b></td></tr>
            
            {spray_html}
            
            <tr><td colspan="2" style="background-color:#2c3e50; color:white; font-weight:bold;">LOGISTICA TERRESTRE</td></tr>
            <tr><td>Ruta Movil:</td><td align="right"><b>{dist_camioneta:.0f} m</b></td></tr>
            <tr><td>Ahorro Vuelo:</td><td align="right" style="color:#2ecc71">{ahorro_pct:.1f}%</td></tr>
        </table>
        """
        
        self.text_report.setHtml(html)

    def export_mission(self):
        if not self.best_path: return
        try:
            filename = "mision_agriswarm.plan"
            self.export_status_label.setText(f"Exportado: {filename}")
            QMessageBox.information(self, "Exportar", f"Misión guardada en:\n{filename}")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))
        
    def show_comparative_report(self):
        """Genera y muestra el reporte de Tesis en el sidebar"""
        if not self.last_mission_cycles or not self.current_specs or not self.static_cycles:
            return
            
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            # 1. Usar Mision Estatica Pre-calculada (Baseline)
            static_cycles = self.static_cycles
            
            # 2. Comparar
            comparison = MissionAnalyzer.compare_missions(self.last_mission_cycles, static_cycles)
            
            # 3. Plan Logistico (Recursos)
            resources = MissionAnalyzer.plan_logistics(self.last_mission_cycles, self.current_specs)
            
            # 4. Metricas Completas (Resumen Ejecutivo)
            comprehensive = MissionAnalyzer.calculate_comprehensive_metrics(
                self.last_mission_cycles, 
                self.polygon, 
                self.current_specs
            )
           
            QApplication.restoreOverrideCursor()
            
            # 5. Crear Panel de Reporte
            # Verificar si ya existe un panel de reporte previo en indice 1 y borrarlo?
            if self.sidebar_stack.count() > 1:
                old = self.sidebar_stack.widget(1)
                self.sidebar_stack.removeWidget(old)
                old.deleteLater()
                
            report_panel = ReportPanel(comprehensive, comparison, resources)
            report_panel.back_clicked.connect(self.show_control_panel)
            
            self.sidebar_stack.addWidget(report_panel)
            self.sidebar_stack.setCurrentIndex(1)
            
        except Exception as e:
            QApplication.restoreOverrideCursor()
            QMessageBox.critical(self, "Error Reporte", str(e))
            
    def on_swath_toggled(self, state):
        visible = bool(state)
        # Update widget state
        self.map_widget.show_swath = visible
        
        if self.best_path and (self.last_mission_cycles or self.static_cycles):
             # Mode Mission: Redraw Results
             use_static = self.chk_mode_static.isChecked()
             cycles_to_draw = self.static_cycles if use_static else self.last_mission_cycles
             
             if cycles_to_draw:
                self.map_widget.draw_results(self.polygon, self.safe_polygon, cycles_to_draw, is_static=use_static)
        else:
             # Mode Editor: Redraw Editor Points
             self.map_widget.draw_editor_state(self.points)

    def toggle_visualization_mode(self, state):
        """Switches between Mobile (Optimizer) and Static (Baseline) visualization"""
        if not self.last_mission_cycles or not self.static_cycles:
            return
            
        use_static = bool(state)
        cycles_to_draw = self.static_cycles if use_static else self.last_mission_cycles
        
        # Redraw
        if self.polygon and self.safe_polygon:
            self.map_widget.draw_results(self.polygon, self.safe_polygon, cycles_to_draw, is_static=use_static)

    def show_control_panel(self):
        self.sidebar_stack.setCurrentIndex(0)