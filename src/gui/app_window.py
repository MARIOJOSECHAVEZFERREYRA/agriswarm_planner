import sys
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QComboBox, QPushButton, QTextEdit, QMessageBox, 
                             QFrame, QSizePolicy)
from PyQt6.QtCore import Qt
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import substring
import math
import datetime

# Imports Propios
from drone_db import DroneDB
from field_io import FieldIO
from algorithms import MarginReducer, BoustrophedonPlanner, GeneticOptimizer
# from decomposition import ConcaveDecomposer # Not currently used in app_window based on previous view?
from geo_utils import GeoUtils
from gui.map_widget import MapWidget
from gui.styles import *

class AgriSwarmApp(QMainWindow):
    def __init__(self, filename=None):
        super().__init__()
        self.filename = filename
        self.setWindowTitle("AgriSwarm GCS - Professional")
        self.setGeometry(100, 100, 1366, 800)
        self.setStyleSheet("background-color: #ecf0f1;")

        # Estado Logico
        self.points = []
        self.polygon = None
        self.current_drone = "DJI Agras T30"
        self.best_path = None
        self.metrics = None
        self.truck_dist = 0

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

        # 2. BARRA LATERAL (Derecha)
        sidebar = QWidget()
        sidebar.setFixedWidth(380) # Un poco mas ancho para el reporte
        sidebar.setStyleSheet(SIDEBAR_STYLE)
        layout.addWidget(sidebar)

        side_layout = QVBoxLayout(sidebar)
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

        self.add_separator(side_layout)

        self.btn_calc = QPushButton("CALCULAR RUTA")
        self.btn_calc.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_calc.setFixedHeight(60)
        self.btn_calc.setStyleSheet(BTN_CALC_STYLE)
        self.btn_calc.clicked.connect(self.run_optimization)
        side_layout.addWidget(self.btn_calc)

        side_layout.addWidget(QLabel("REPORTE TECNICO"))
        self.text_report = QTextEdit()
        self.text_report.setReadOnly(True)
        # Habilitar HTML
        self.text_report.setAcceptRichText(True)
        self.text_report.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        side_layout.addWidget(self.text_report)

        self.btn_export = QPushButton("EXPORTAR .PLAN")
        self.btn_export.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_export.setFixedHeight(45)
        self.btn_export.setStyleSheet(BTN_EXPORT_STYLE)
        self.btn_export.clicked.connect(self.export_mission)
        self.btn_export.setEnabled(False)
        side_layout.addWidget(self.btn_export)

        # Inicializar
        self.map_widget.clear_map()

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
        self.text_report.clear()
        self.map_widget.clear_map()

    def load_field(self):
        if not self.filename: return
        try:
            poly = FieldIO.load_field(self.filename)
            self.points = list(poly.exterior.coords)[:-1]
            self.best_path = None
            self.map_widget.draw_editor_state(self.points)
            self.text_report.setText("Campo cargado correctamente.")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def on_drone_changed(self, text):
        self.current_drone = text
        self.text_report.setText(f"Dron activo: {text}")

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
        
        try:
            planner = BoustrophedonPlanner(spray_width=real_swath)
            optimizer = GeneticOptimizer(planner, pop_size=50, generations=30)
            best_angle, self.best_path, self.metrics = optimizer.optimize(campo_seguro)
            
            px = [p[0] for p in self.best_path]
            py = [p[1] for p in self.best_path]
            start = Point(px[0], py[0])
            end = Point(px[-1], py[-1])
            road_poly = self.polygon.buffer(5.0, join_style=2)
            truck_path, self.truck_dist = self.calculate_truck_route(road_poly.exterior, start, end)
            
            truck_coords = list(truck_path.coords) if truck_path else []

            self.map_widget.draw_results(self.polygon, campo_seguro, self.best_path, truck_coords)
            
            # GENERAR REPORTE DETALLADO
            self.generate_report(self.metrics, specs, self.polygon, self.truck_dist)
            self.btn_export.setEnabled(True)

        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def calculate_truck_route(self, ring, p_start, p_end):
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
            coords = list(part1.coords) + list(part2.coords)[1:]
            return LineString(coords), dist_B

    def generate_report(self, metrics, specs, campo, truck_dist):
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
            GeoUtils.export_qgc_mission(self.best_path, filename)
            QMessageBox.information(self, "OK", f"Mision exportada:\n{filename}")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))