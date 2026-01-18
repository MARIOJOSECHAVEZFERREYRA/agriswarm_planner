from PyQt6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsTextItem, QGraphicsEllipseItem, QGraphicsRectItem
from PyQt6.QtCore import Qt, QPointF, pyqtSignal, QRectF
from PyQt6.QtGui import QPen, QBrush, QColor, QFont, QPainter, QPolygonF, QWheelEvent, QMouseEvent, QPainterPath
import math

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
        
        # Estado de arrastre interno
        self.dragging_point_index = None

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

    def draw_results(self, polygon_geom, safe_geom, mission_cycles):
        """
        Dibuja los resultados de la mision segmentada.
        :param mission_cycles: Lista de dicts [{'type': 'work', 'path': [], 'truck_path': ...}, ...]
        """
        self.clear_map()
        
        if polygon_geom:
            poly_q = QPolygonF([QPointF(x, y) for x, y in polygon_geom.exterior.coords])
            brush = QBrush(QColor(46, 204, 113, 50))
            pen = QPen(QColor('#27ae60'))
            pen.setWidth(3)
            pen.setCosmetic(True)
            self.scene.addPolygon(poly_q, pen, brush).setZValue(0)
            
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
        for cycle in mission_cycles:
            path = cycle.get('path', [])
            truck_path_list = cycle.get('truck_path_coords', []) # Expecting list of coords
            
            if not path: continue
            
            # 1. Dibujar Ruta Vuelo
            col = colors[cycle_idx % len(colors)]
            
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
                tpath.moveTo(truck_path_list[0][0], truck_path_list[0][1])
                for p in truck_path_list[1:]:
                    tpath.lineTo(p[0], p[1])
                
                pen_t = QPen(QColor('#e67e22'))
                pen_t.setStyle(Qt.PenStyle.DashLine)
                pen_t.setWidth(3)
                pen_t.setCosmetic(True)
                self.scene.addPath(tpath, pen_t).setZValue(5)
                
                # Marcador Truck
                # self.draw_mission_marker(truck_path_list[0][0], truck_path_list[0][1], '#d35400', "T")
            
            cycle_idx += 1
            
        # Marcador Inicio Global
        if mission_cycles and mission_cycles[0].get('path'):
            p0 = mission_cycles[0]['path'][0]
            self.draw_mission_marker(p0[0], p0[1], '#2ecc71', "S")

    def draw_mission_marker(self, x, y, color, text):
        item = MissionMarkerItem(x, y, text, color)
        self.scene.addItem(item)

    def draw_labels(self, points):
        if len(points) < 2: return
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[0] if i == len(points)-1 else points[i+1]
            
            dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            mid_x = (p1[0] + p2[0]) / 2
            mid_y = (p1[1] + p2[1]) / 2
            
            self.draw_floating_label(mid_x, mid_y, f"{dist:.1f} m")

    def draw_floating_label(self, x, y, text, is_area=False):
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
        
        t.setPos(x, y)
        t.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        t.setZValue(200) # ¡MUY ALTO PARA VERSE SIEMPRE!
        self.scene.addItem(t)

    # --- EVENTOS ---
    def wheelEvent(self, event: QWheelEvent):
        factor = 1.2
        if event.angleDelta().y() > 0: self.scale(factor, factor)
        else: self.scale(1/factor, 1/factor)

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

    def mouseMoveEvent(self, event: QMouseEvent):
        super().mouseMoveEvent(event)
        # Si estamos arrastrando un punto, emitir la señal para actualizar geometria
        if self.dragging_point_index is not None:
            scene_pos = self.mapToScene(event.pos())
            self.point_moved.emit(self.dragging_point_index, scene_pos.x(), scene_pos.y())

    def mouseReleaseEvent(self, event: QMouseEvent):
        super().mouseReleaseEvent(event)
        self.dragging_point_index = None
        self.setDragMode(QGraphicsView.DragMode.NoDrag)