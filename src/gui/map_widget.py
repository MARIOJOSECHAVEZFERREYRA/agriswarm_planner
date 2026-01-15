from PyQt6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsTextItem, QGraphicsEllipseItem, QGraphicsRectItem
from PyQt6.QtCore import Qt, QPointF, pyqtSignal, QRectF
from PyQt6.QtGui import QPen, QBrush, QColor, QFont, QPainter, QPolygonF, QWheelEvent, QMouseEvent, QPainterPath
import math

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

    def draw_results(self, polygon_geom, safe_geom, path, truck_path):
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

        if path:
            qpath = QPainterPath()
            qpath.moveTo(path[0][0], path[0][1])
            for p in path[1:]:
                qpath.lineTo(p[0], p[1])
            
            pen = QPen(QColor('#2980b9'))
            pen.setWidth(2)
            pen.setCosmetic(True)
            self.scene.addPath(qpath, pen).setZValue(10)
            
            self.draw_mission_marker(path[0][0], path[0][1], '#2ecc71', "S")
            self.draw_mission_marker(path[-1][0], path[-1][1], '#e74c3c', "E")

        if truck_path:
            tpath = QPainterPath()
            tpath.moveTo(truck_path[0][0], truck_path[0][1])
            for p in truck_path[1:]:
                tpath.lineTo(p[0], p[1])
            pen = QPen(QColor('#f39c12'))
            pen.setStyle(Qt.PenStyle.DashLine)
            pen.setWidth(2)
            pen.setCosmetic(True)
            self.scene.addPath(tpath, pen).setZValue(5)

    def draw_mission_marker(self, x, y, color, text):
        r = 8
        el = QGraphicsEllipseItem(-r, -r, r*2, r*2)
        el.setBrush(QBrush(QColor(color)))
        el.setPen(QPen(Qt.GlobalColor.white, 2))
        el.setPos(x, y)
        el.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        el.setZValue(50)
        self.scene.addItem(el)
        
        font = QFont("Arial", 8, QFont.Weight.Bold)
        t = QGraphicsTextItem(text)
        t.setFont(font)
        t.setDefaultTextColor(Qt.GlobalColor.white)
        # Centrar manualmente la letra
        t.setPos(x - 4, y - 10) 
        t.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        t.setZValue(51)
        self.scene.addItem(t)

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