from PyQt6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsTextItem, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsSimpleTextItem, QGraphicsItemGroup, QGraphicsPolygonItem
from PyQt6.QtCore import Qt, QPointF, pyqtSignal, QRectF
from PyQt6.QtGui import QPen, QBrush, QColor, QFont, QPainter, QPolygonF, QWheelEvent, QMouseEvent, QPainterPath
import math
from shapely.geometry import LineString, Polygon as ShapelyPoly

class MissionMarkerItem(QGraphicsItem):
    """
    Marcador personalizado que ignora transformaciones (zoom) para mantener
    tamaño constante en pixeles, con etiqueta de texto clara.
    """
    def __init__(self, x, y, label, color, type="default"):
        super().__init__()
        self.setPos(x, y)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.setZValue(100) # Siempre visible
        self.label = label
        self.color = QColor(color)
        self.text_color = QColor("white")
        
        # Mapeo de nombres cortos a largos
        if label == "S": self.full_text = "START"
        elif label == "E": self.full_text = "END"
        elif label.startswith("R"): self.full_text = label.replace("R", "R")
        else: self.full_text = label
        
        # Estilos especificos
        if label == "S": self.color = QColor("#2ecc71") # Green
        if label == "E": self.color = QColor("#e74c3c") # Red
        if label.startswith("R"): self.color = QColor("#f39c12") # Orange for truck stops

    def boundingRect(self):
        # Area de dibujo aproximada (marker + text bubble)
        return QRectF(-10, -50, 200, 60)

    def paint(self, painter, option, widget):
        # 1. Dibujar Punto/Pin
        r = 6
        painter.setBrush(QBrush(self.color))
        painter.setPen(QPen(Qt.GlobalColor.white, 2))
        painter.drawEllipse(QPointF(0, 0), r, r)
        
        # 2. Dibujar "Callout" (Burbuja de texto)
        # Offset en PIXELES (fijo, no depende del zoom)
        offset_x = 10
        offset_y = -10
        
        font = QFont("Segoe UI", 9, QFont.Weight.Bold)
        painter.setFont(font)
        fm = painter.fontMetrics()
        
        text_w = fm.horizontalAdvance(self.full_text)
        text_h = fm.height()
        pad = 6
        
        # Rectangulo de fondo
        rect_x = offset_x
        rect_y = offset_y - text_h - pad
        rect_w = text_w + (pad * 2)
        rect_h = text_h + pad
        
        # Linea conectora (opcional)
        # painter.setPen(QPen(self.color, 1))
        # painter.drawLine(0, 0, rect_x, rect_y + rect_h)
        
        # Caja
        path = QPainterPath()
        path.addRoundedRect(QRectF(rect_x, rect_y, rect_w, rect_h), 4, 4)
        
        painter.setBrush(QBrush(self.color))
        painter.setPen(QPen(Qt.GlobalColor.white, 1))
        painter.drawPath(path)
        
        # Texto
        painter.setPen(self.text_color)
        painter.drawText(int(rect_x + pad), int(rect_y + text_h - 2), self.full_text)

class MapWidget(QGraphicsView):
    # Señales
    map_clicked = pyqtSignal(float, float)       # Click en vacio (Añadir punto)
    map_right_clicked = pyqtSignal(float, float) # Click derecho
    point_moved = pyqtSignal(int, float, float)  # Arrastrar punto (indice, nueva_x, nueva_y)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_app = parent
        
        # Estado
        self.original_polygon = None
        self.truck_path_coords = []     # Lista de coords de la camioneta
        self.mission_cycles = None      # Lista de ciclos (path + color + markers)
        self.recharge_markers = []      # Lista de dicts {lat, lon, label}
        self.swath_polygons = []        # Lista de poligonos de cobertura
        self.show_swath = True          # Visibility toggle
        self.scene = QGraphicsScene(self)
        self.scene.setSceneRect(-100000, -100000, 200000, 200000)
        self.setScene(self.scene)
        
        # Configuración
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setBackgroundBrush(QBrush(QColor("#ffffff")))
        
        self.scale(10, -10)
        
        # Mouse Tracking for Hover Labels
        self.setMouseTracking(True)
        self.hover_group = None
        self.hover_text = None
        self.hover_bg = None
        
        # Estado de arrastre interno
        self.dragging_point_index = None

        # State
        self.zoom_level = 1.0 # Note: m11 tracks internal scale too, but we keep this for legacy logic if any
        self.pan_start = QPointF(0, 0)
        self.is_panning = False
        self.show_swath = True
        
        # Cache for redraw
        self.last_polygon_geom = None
        self.last_safe_geom = None
        self.last_mission_cycles = None

    def set_swath_visibility(self, visible: bool):
        self.show_swath = visible
        if self.last_mission_cycles:
             self.draw_results(self.last_polygon_geom, self.last_safe_geom, self.last_mission_cycles)
    
    def drawBackground(self, painter, rect):
        """Rejilla Dinámica"""
        super().drawBackground(painter, rect)
        scale = self.transform().m11()
        if scale <= 0: scale = 1.0
        
        step_pixel = 100
        step_world = step_pixel / scale
        exponent = math.floor(math.log10(step_world))
        fraction = step_world / (10**exponent)
        
        if fraction < 2: grid_step = 2 * 10**exponent
        elif fraction < 5: grid_step = 5 * 10**exponent
        else: grid_step = 10 * 10**exponent
            
        painter.setPen(QPen(QColor(240, 240, 240), 0))
        left = int(rect.left()) - (int(rect.left()) % int(grid_step))
        top = int(rect.top()) - (int(rect.top()) % int(grid_step))
        
        x = left
        while x < rect.right():
            painter.drawLine(int(x), int(rect.top()), int(x), int(rect.bottom()))
            x += grid_step
        y = top
        while y < rect.bottom():
            painter.drawLine(int(rect.left()), int(y), int(rect.right()), int(y))
            y += grid_step

    def drawForeground(self, painter, rect):
        """Barra de Escala"""
        scale = self.transform().m11()
        if scale <= 0: return

        target_width_px = 120
        world_width = target_width_px / scale
        exponent = math.floor(math.log10(world_width))
        fraction = world_width / (10**exponent)
        
        if fraction < 2: nice_world = 1 * 10**exponent
        elif fraction < 5: nice_world = 2 * 10**exponent
        else: nice_world = 5 * 10**exponent
        
        bar_width_px = nice_world * scale
        
        vp = self.viewport().rect()
        margin = 20
        start_x = vp.width() - bar_width_px - margin
        start_y = vp.height() - margin - 10 
        
        painter.resetTransform()
        pen = QPen(QColor("black"))
        pen.setWidth(2)
        painter.setPen(pen)
        
        h = 6
        painter.drawLine(int(start_x), int(start_y - h), int(start_x), int(start_y))
        painter.drawLine(int(start_x), int(start_y), int(start_x + bar_width_px), int(start_y))
        painter.drawLine(int(start_x + bar_width_px), int(start_y), int(start_x + bar_width_px), int(start_y - h))
        
        text = f"{int(nice_world)} m"
        font = QFont("Segoe UI", 9, QFont.Weight.Bold)
        painter.setFont(font)
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(text)
        painter.drawText(int(start_x + (bar_width_px - tw)/2), int(start_y - 8), text)

    # --- DIBUJO ---

    def clear_map(self):
        self.scene.clear()
        self.hover_group = None
        self.hover_text = None
        self.hover_bg = None
        
        # Reset Geometries to prevent Ghost Hover detection
        self.last_polygon_geom = None
        self.last_safe_geom = None
        self.mission_cycles = None 
        
        # Cache for overlapping labels
        self.label_cache = {}

    def draw_editor_state(self, points):
        """Dibuja polígono editable"""
        self.clear_map()
        if not points: return

        # 1. Líneas
        if len(points) > 1:
            path = QPainterPath()
            path.moveTo(*points[0])
            for p in points[1:]:
                path.lineTo(*p)
            
            pen = QPen(QColor('#4285F4'))
            pen.setWidth(3)
            pen.setCosmetic(True)
            self.scene.addPath(path, pen).setZValue(1)
            
            if len(points) > 2:
                pen_dash = QPen(QColor('#4285F4'))
                pen_dash.setStyle(Qt.PenStyle.DashLine)
                pen_dash.setWidth(2)
                pen_dash.setCosmetic(True)
                self.scene.addLine(points[-1][0], points[-1][1], points[0][0], points[0][1], pen_dash).setZValue(1)

        # 2. Puntos (Interactivos)
        for i, p in enumerate(points):
            self.draw_point_marker(p[0], p[1], index=i)

        # 3. Etiquetas
        self.draw_labels(points)

    def draw_point_marker(self, x, y, index):
        """Punto interactivo que sabe su índice"""
        radius = 6 
        ellipse = QGraphicsEllipseItem(-radius, -radius, radius * 2, radius * 2)
        
        ellipse.setBrush(QBrush(QColor("white")))
        pen = QPen(QColor("#555555"))
        pen.setWidth(2)
        ellipse.setPen(pen)
        
        ellipse.setPos(x, y)
        ellipse.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        ellipse.setZValue(100) # Capa alta para capturar clics
        
        # GUARDAR INDICE EN EL ITEM PARA IDENTIFICARLO AL ARRASTRAR
        ellipse.setData(0, index) 
        
        self.scene.addItem(ellipse)

    def draw_results(self, polygon_geom, safe_geom, mission_cycles, is_static=False):
        """
        Dibuja los resultados de la mision segmentada.
        :param mission_cycles: Lista de dicts [{'type': 'work', 'path': [], 'truck_path': ...}, ...]
        :param is_static: Boolean, si es True activamos visualizaciones especificas (labels de retorno).
        """

        self.last_polygon_geom = polygon_geom
        self.last_safe_geom = safe_geom
        self.last_mission_cycles = mission_cycles
        
        self.clear_map()
        
        if polygon_geom:
            poly_q = QPolygonF([QPointF(x, y) for x, y in polygon_geom.exterior.coords])
            brush = QBrush(QColor(46, 204, 113, 50))
            pen = QPen(QColor('#27ae60'))
            pen.setWidth(3)
            pen.setCosmetic(True)
            self.scene.addPolygon(poly_q, pen, brush).setZValue(0)
            
            # 3. Etiquetas Permanentes (Restaurado)
            self.draw_labels(list(polygon_geom.exterior.coords)[:-1])
            
            centroid = polygon_geom.centroid
            area_ha = polygon_geom.area / 10000.0
            self.draw_floating_label(centroid.x, centroid.y, f"{area_ha:.2f} ha", is_area=True)

        if safe_geom:
            poly_s = QPolygonF([QPointF(x, y) for x, y in safe_geom.exterior.coords])
            pen_s = QPen(QColor('#e74c3c'))
            pen_s.setStyle(Qt.PenStyle.DashLine)
            pen_s.setWidth(2)
            pen_s.setCosmetic(True)
            self.scene.addPolygon(poly_s, pen_s, QBrush(Qt.BrushStyle.NoBrush)).setZValue(2)

        # Paleta de colores para ciclos
        colors = ['#2980b9', '#8e44ad', '#16a085', '#d35400', '#2c3e50', '#c0392b']
        
        cycle_idx = 0
        drawn_segments_history = {} # Key: ( (x1,y1), (x2,y2) ) -> Count
        for cycle in mission_cycles:
            path = cycle.get('path', [])
            truck_path_list = cycle.get('truck_path_coords', []) # Expecting list of coords
            
            if not path: continue
            
            # 1. Dibujar Ruta Vuelo con ANCHO REAL (SWATH)
            col = colors[cycle_idx % len(colors)]
            swath_width = cycle.get('swath_width', 5.0) # Default 5m
            
            # NUEVA LOGICA: Usar metadatos de segmentos si existen
            segments_meta = cycle.get('segments', [])
            
            if segments_meta:
                # Iterar por segmento individual para decidir si dibujamos "gordo" o "fino"
                for seg in segments_meta:
                    p1 = seg['p1']
                    p2 = seg['p2']
                    is_spraying = seg['spraying']
                    
                    # Dibujar Linea Central (Siempre, pero estilo varia?)
                    # Mejor: Si es spraying -> Buffer + Linea fina centro
                    # Si no es spraying -> Solo Linea fina (o punteada?)
                    
                    pen_seg = QPen(QColor(col))
                    
                    # 1. Linea base
                    qseg = QPainterPath()
                    qseg.moveTo(p1[0], p1[1])
                    qseg.lineTo(p2[0], p2[1])
                    
                    if is_spraying:
                        pen_seg.setWidth(2)
                        pen_seg.setStyle(Qt.PenStyle.SolidLine)
                    else:
                        # Transit: Dashed + ARROWS
                        pen_seg.setStyle(Qt.PenStyle.DashLine)
                        pen_seg.setWidth(2)
                        
                        # Draw Arrow to show direction
                        self.draw_arrow(p1, p2, col)

                    pen_seg.setCosmetic(True)
                    self.scene.addPath(qseg, pen_seg).setZValue(10)
                    
                    # 2. Buffer (Solo si Spraying y Habilitado)
                    if self.show_swath and is_spraying:
                        line_geom = LineString([p1[:2], p2[:2]])
                        swath_poly = line_geom.buffer(swath_width / 2.0, cap_style=2, join_style=2)
                        
                        if swath_poly.geom_type == 'Polygon':
                            polys = [swath_poly]
                        else:
                            polys = swath_poly.geoms
                            
                        for poly in polys:
                             qpoly = QPolygonF([QPointF(x, y) for x, y in poly.exterior.coords])
                             c = QColor(col)
                             c.setAlpha(150) # Mas visible! (antes 90)
                             brush = QBrush(c)
                             pen_b = QPen(Qt.PenStyle.NoPen)
                             self.scene.addPolygon(qpoly, pen_b, brush).setZValue(9)

                    # 3. STATIC MODE: Label Return Segments (Non-Spraying)
                    if is_static and not is_spraying:
                        d_seg = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
                        if d_seg > 10.0: # Only significant return paths
                            mx = (p1[0] + p2[0]) / 2
                            my = (p1[1] + p2[1]) / 2
                            self.draw_floating_label(mx, my, f"{d_seg:.0f} m")

            else:
                # FALLBACK (Codigo Viejo)
                # A. Dibujar Buffer Completo
                if len(path) > 1:
                    flight_line = LineString(path)
                    swath_poly = flight_line.buffer(swath_width / 2.0, cap_style=2, join_style=2)
                    
                    if swath_poly.geom_type == 'Polygon':
                        polys = [swath_poly]
                    else:
                        polys = swath_poly.geoms # MultiPolygon
                        
                    for poly in polys:
                        qpoly = QPolygonF([QPointF(x, y) for x, y in poly.exterior.coords])
                        c = QColor(col)
                        c.setAlpha(80) 
                        brush = QBrush(c)
                        pen_b = QPen(Qt.PenStyle.NoPen)
                        self.scene.addPolygon(qpoly, pen_b, brush).setZValue(9)
    
                # B. Dibujar Linea Central
                qpath = QPainterPath()
                qpath.moveTo(path[0][0], path[0][1])
                for p in path[1:]:
                    qpath.lineTo(p[0], p[1])
                
                pen = QPen(QColor(col))
                pen.setWidth(2)
                pen.setCosmetic(True)
                self.scene.addPath(qpath, pen).setZValue(10)
            
            # Marcadores de Inicio/Fin de ciclo
            label_start = "S" if cycle_idx == 0 else f"R{cycle_idx}"
            label_end = "E" if cycle_idx == len(mission_cycles)-1 else f"R{cycle_idx+1}"
            
            # Solo dibujar si no se solapan demasiado o logica especifica
            # self.draw_mission_marker(path[0][0], path[0][1], col, label_start)
            # El punto final de un ciclo es el punto de recarga del siguiente usualmente
            self.draw_mission_marker(path[-1][0], path[-1][1], '#e74c3c', label_end) # Rojo para fin/recarga

            # 2. Dibujar Ruta Camion (si existe para este ciclo)
            if truck_path_list and len(truck_path_list) > 1:
                tpath = QPainterPath()
                
                # --- COLOR GRADIENT LOGIC ---
                # En lugar de desplazar espacialmente, cambiamos el color para indicar "capas" de tiempo.
                # Base: Naranja (#e67e22). 
                # Cada vez que pasamos por el mismo sitio, oscurecemos o cambiamos el tono.
                
                p_start = truck_path_list[0]
                p_end = truck_path_list[-1]
                
                # Relajar la precision para detectar solapamientos
                # Usar enteros (metros) es suficiente para saber si es "el mismo tramo"
                # O incluso round a 0 decimales.
                k1 = (int(p_start[0]), int(p_start[1]))
                k2 = (int(p_end[0]), int(p_end[1]))
                
                # Ordenar para que A->B sea igual que B->A
                segment_key = tuple(sorted((k1, k2)))
                
                # Check fuzzy match against existing keys?
                # Por ahora probamos con claves enteras rigidas.
                
                overlap_count = drawn_segments_history.get(segment_key, 0)
                drawn_segments_history[segment_key] = overlap_count + 1
                
                # Construir Path (Directo, CENTRADO, sin offset espacial)
                tpath.moveTo(truck_path_list[0][0], truck_path_list[0][1])
                for p in truck_path_list[1:]:
                    tpath.lineTo(p[0], p[1])
                
                # Calcular Color - PALETA DE ALTO CONTRASTE
                # 0: Naranja (Base)
                # 1: Rojo (Ida y Vuelta / Trafico Medio)
                # 2: Morado (Trafico Alto)
                # 3+: Negro (Saturado)
                
                if overlap_count == 0:
                    pen_color = QColor('#e67e22') # Naranja
                    width = 3
                elif overlap_count == 1:
                    pen_color = QColor('#e74c3c') # Rojo Brillante
                    width = 3
                elif overlap_count == 2:
                    pen_color = QColor('#8e44ad') # Morado
                    width = 4
                else:
                    pen_color = QColor('#000000') # Negro
                    width = 4

                pen_t = QPen(pen_color) 
                pen_t.setStyle(Qt.PenStyle.DashLine)
                pen_t.setWidth(width) 
                pen_t.setCosmetic(True)
                self.scene.addPath(tpath, pen_t).setZValue(5 + overlap_count) # Poner encima las nuevas capas
            


            cycle_idx += 1
            
        # Marcador Inicio Global
        if mission_cycles and mission_cycles[0].get('path'):
            p0 = mission_cycles[0]['path'][0]
            self.draw_mission_marker(p0[0], p0[1], '#2ecc71', "S")

    def draw_mission_marker(self, x, y, color, text):
        item = MissionMarkerItem(x, y, text, color)
        self.scene.addItem(item)

    def draw_arrow(self, p1, p2, color):
        """Dibuja una flecha en el punto medio del segmento para indicar direccion (Unidades de Mapa)."""
        mx = (p1[0] + p2[0]) / 2
        my = (p1[1] + p2[1]) / 2
        
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.sqrt(dx*dx + dy*dy)
        if length < 15.0: return # Don't draw arrows on short segments

        angle = math.atan2(dy, dx)
        
        # Geometry in PIXELS (Fixed Screen Size)
        psize = 12
        
        # Position Logic (Relative):
        # Place arrow at 60% of the segment (Slightly forward).
        # This resolves overlap (0.6 vs 0.4) without pushing it to the ends.
        t = 0.60
            
        mx = p1[0] + dx * t
        my = p1[1] + dy * t
        
        # Triangle pointing Right (+X), Centered on Centroid (0,0)
        # Shifted coords to align visual center with pivot
        head_px = QPolygonF([
            QPointF(-4, -4),
            QPointF(8, 0),
            QPointF(-4, 4)
        ])
        
        arrow_item = QGraphicsPolygonItem(head_px)
        arrow_item.setBrush(QBrush(QColor(color)))
        arrow_item.setPen(QPen(Qt.PenStyle.NoPen))
        
        # Transform 
        # 1. Rotation (Degrees)
        # NOTE: View uses scale(10, -10) (Y-flip). 
        # Screen Y is Down, Scene Y is Up. 
        # Scene Angle +45 (Up-Right) -> Screen Angle -45 (Up-Right).
        # Must NEGATE the angle for ItemIgnoresTransformations items.
        arrow_item.setRotation(-math.degrees(angle))
        
        # 2. Position (Scene Coords)
        arrow_item.setPos(mx, my)
        
        # 3. Ignore Zoom (Constant Pixel Size)
        arrow_item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        
        arrow_item.setZValue(12) 
        self.scene.addItem(arrow_item)

    def draw_labels(self, points):
        if len(points) < 2: return
        
        # Calc Centroid
        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)
        
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[0] if i == len(points)-1 else points[i+1]
            
            dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            mid_x = (p1[0] + p2[0]) / 2
            mid_y = (p1[1] + p2[1]) / 2
            
            # Vector Edge
            vx = p2[0] - p1[0]
            vy = p2[1] - p1[1]
            
            # Normal (rotate 90 deg)
            nx = -vy
            ny = vx
            
            # Check direction relative to centroid
            # Vector Centroid -> Midpoint
            v_cm_x = mid_x - cx
            v_cm_y = mid_y - cy
            
            # Dot Product
            dot = nx * v_cm_x + ny * v_cm_y
            
            if dot < 0:
                nx = -nx
                ny = -ny
            
            angle = math.atan2(ny, nx)
            
            self.draw_floating_label(mid_x, mid_y, f"{dist:.1f} m", angle=angle)

    def draw_cycle_label(self, x, y, text):
        """Dibuja una etiqueta pequeña para el ciclo (C1, C2...)"""
        # Convert newlines to breaks
        html_text = text.replace('\n', '<br>')
        
        qtext = QGraphicsTextItem()
        font = QFont("Arial", 10, QFont.Weight.Bold)
        qtext.setFont(font)
        
        # Centering via HTML
        qtext.setHtml(f"<div style='text-align: center;'>{html_text}</div>")
        
        # Center item on origin
        rect = qtext.boundingRect()
        rect_center_off_x = -rect.width() / 2
        rect_center_off_y = -rect.height() / 2
        qtext.setPos(rect_center_off_x, rect_center_off_y)
        
        # Background
        bg = QGraphicsRectItem(rect)
        bg.setBrush(QBrush(QColor(255, 255, 255, 200))) 
        bg.setPen(QPen(Qt.PenStyle.NoPen))
        bg.setPos(rect_center_off_x, rect_center_off_y)
        
        group = QGraphicsItemGroup()
        group.addToGroup(bg)
        group.addToGroup(qtext)
        
        group.setPos(x, y) # Position at centroid
        group.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations) # Scale independent
        group.setZValue(20) # Top
        self.scene.addItem(group)

    def draw_floating_label(self, x, y, text, is_area=False, angle=None):
        # Check cache for overlaps (only for distance labels, to show x2)
        if not is_area:
             key = (round(x, 1), round(y, 1))
             if hasattr(self, 'label_cache') and key in self.label_cache:
                 existing_t, orig_text = self.label_cache[key]
                 if text == orig_text:
                     # Merge and update existing label
                     new_text = f"{text} (x2)"
                     
                     border_col = "#bdc3c7"
                     bg_col = "rgba(255, 255, 255, 0.9)"
                     existing_t.setHtml(f"<div style='background-color: {bg_col}; border: 1px solid {border_col}; padding: 2px 4px; border-radius: 4px;'>{new_text}</div>")
                     
                     # Re-center (size changed)
                     rect = existing_t.boundingRect()
                     existing_t.setPos(-rect.width()/2, -rect.height()/2)
                     return

        t = QGraphicsTextItem(text)
        font = QFont("Segoe UI", 10 if is_area else 9, QFont.Weight.Bold)
        t.setFont(font)
        
        # Color verde oscuro para area, gris oscuro para distancias
        color = "#145A32" if is_area else "#2c3e50"
        t.setDefaultTextColor(QColor(color))
        
        # Fondo estilo etiqueta
        border_col = "#27ae60" if is_area else "#bdc3c7"
        bg_col = "rgba(255, 255, 255, 0.9)"
        
        t.setHtml(f"<div style='background-color: {bg_col}; border: 1px solid {border_col}; padding: 2px 4px; border-radius: 4px;'>{text}</div>")
        
        # Center the text item on its origin 0,0
        rect = t.boundingRect()
        t.setPos(-rect.width()/2, -rect.height()/2)

        # Cache it
        if not is_area and hasattr(self, 'label_cache'):
            self.label_cache[key] = (t, text)

        # Container Group
        group = QGraphicsItemGroup()
        group.addToGroup(t)
        
        group.setPos(x, y)
        group.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        group.setZValue(20)
        
        if angle is not None:
            # Offset in pixels
            offset = 45 # Aumentado para mayor separación
            dx = offset * math.cos(angle)
            dy = -offset * math.sin(angle) # Negate Y because View is Y-flipped (Scene Up = Screen Y-)
            # Apply offset to text item inside the scale-invariant group
            t.setPos(t.x() + dx, t.y() + dy)
        
        self.scene.addItem(group)
        



    # --- EVENTOS ---
    def wheelEvent(self, event: QWheelEvent):
        factor = 1.2
        if event.angleDelta().y() > 0: self.scale(factor, factor)
        else: self.scale(1/factor, 1/factor)

    def mouseMoveEvent(self, event: QMouseEvent):
        super().mouseMoveEvent(event)
        
        # 1. Logic for Dragging Points (Priority)
        if self.dragging_point_index is not None:
            scene_pos = self.mapToScene(event.pos())
            self.point_moved.emit(self.dragging_point_index, scene_pos.x(), scene_pos.y())
            return # Skip hover if dragging

        if not self.last_polygon_geom:
            return

        scene_pos = self.mapToScene(event.pos())
        p_mouse = (scene_pos.x(), scene_pos.y())
        
        min_dist = float('inf')
        closest_edge = None # (p1, p2, dist_len, angle)
        
        coords = list(self.last_polygon_geom.exterior.coords)
        if len(coords) < 2: return
        
        for i in range(len(coords)-1):
            p1 = coords[i]
            p2 = coords[i+1]
            
            # Distance Point to Segment
            px, py = p2[0]-p1[0], p2[1]-p1[1]
            norm = px*px + py*py
            if norm == 0: continue
            
            u = ((p_mouse[0] - p1[0]) * px + (p_mouse[1] - p1[1]) * py) / norm
            u = max(min(u, 1), 0)
            
            x = p1[0] + u * px
            y = p1[1] + u * py
            
            dx = x - p_mouse[0]
            dy = y - p_mouse[1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < min_dist:
                min_dist = dist
                edge_len = math.sqrt(norm)
                
                # Outward Angle Calc
                mx, my = (p1[0]+p2[0])/2, (p1[1]+p2[1])/2
                cx, cy = self.last_polygon_geom.centroid.x, self.last_polygon_geom.centroid.y
                vx, vy = p2[0]-p1[0], p2[1]-p1[1]
                nx, ny = -vy, vx
                if nx*(mx-cx) + ny*(my-cy) < 0: nx, ny = -nx, -ny
                angle = math.atan2(ny, nx)
                
                closest_edge = ((mx, my), edge_len, angle)

        threshold = 5.0 # meters
        
        if min_dist < threshold and closest_edge:
            mid, length, ang = closest_edge
            text = f"{length:.1f} m"
            self.update_hover_label(mid[0], mid[1], text, ang)
        else:
            if self.hover_group: self.hover_group.setVisible(False)

    def update_hover_label(self, x, y, text, angle):
        if not self.hover_group:
            self.hover_text = QGraphicsSimpleTextItem()
            self.hover_text.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            self.hover_bg = QGraphicsRectItem()
            self.hover_bg.setBrush(QBrush(QColor(255, 255, 255, 220)))
            self.hover_bg.setPen(QPen(Qt.PenStyle.NoPen))
            
            self.hover_group = QGraphicsItemGroup()
            self.hover_group.addToGroup(self.hover_bg)
            self.hover_group.addToGroup(self.hover_text)
            self.hover_group.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
            self.hover_group.setZValue(200)
            self.scene.addItem(self.hover_group)
            
        self.hover_group.setVisible(True)
        self.hover_text.setText(text)
        
        # Layout
        r = self.hover_text.boundingRect()
        pad = 4
        self.hover_bg.setRect(r.x()-pad, r.y()-pad, r.width()+2*pad, r.height()+2*pad)
        
        # Center in Group
        off_x = -r.width()/2
        off_y = -r.height()/2
        self.hover_text.setPos(off_x, off_y)
        self.hover_bg.setPos(off_x, off_y)
        
        # Position with Angle Offset
        offset_px = 25
        dx = offset_px * math.cos(angle)
        dy = offset_px * math.sin(angle)
        
        # Since group ignores transform, x/y are scene pos, but internal matrix is pixels.
        # So we can't just add dx to x? 
        # Actually ItemIgnoresTransformations anchors the item at pos() in scene, 
        # and draws it with identity transform (pixels) at that point.
        # But if we want the TEXT to be offset by pixels, we move the item (or group) contents?
        # NO. We move the group anchor? No, group anchor is scene coord.
        # We must add pixel offset to the child items!
        
        # Reset local pos to center
        self.hover_text.setPos(off_x + dx, off_y + dy)
        self.hover_bg.setPos(off_x + dx, off_y + dy)
        
        self.hover_group.setPos(x, y)

    def mousePressEvent(self, event: QMouseEvent):
        # 1. Detectar si hicimos click en un PUNTO existente
        scene_pos = self.mapToScene(event.pos())
        item = self.scene.itemAt(scene_pos, self.transform())
        
        if event.button() == Qt.MouseButton.LeftButton:
            # Si es un punto (tiene datos), iniciamos arrastre
            if isinstance(item, QGraphicsEllipseItem) and item.data(0) is not None:
                self.dragging_point_index = item.data(0)
                self.setDragMode(QGraphicsView.DragMode.NoDrag) # Bloquear otros drags
                return

            if event.modifiers() == Qt.KeyboardModifier.ControlModifier:
                self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
                super().mousePressEvent(event)
            else:
                self.map_clicked.emit(scene_pos.x(), scene_pos.y())
                
        elif event.button() == Qt.MouseButton.RightButton:
            self.map_right_clicked.emit(scene_pos.x(), scene_pos.y())



    def mouseReleaseEvent(self, event: QMouseEvent):
        super().mouseReleaseEvent(event)
        self.dragging_point_index = None
        self.setDragMode(QGraphicsView.DragMode.NoDrag)