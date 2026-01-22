from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QTableWidget, QTableWidgetItem, 
    QHeaderView, QFrame, QPushButton, QWidget, QTabWidget
)
from PyQt6.QtGui import QColor, QFont
from PyQt6.QtCore import Qt
from gui.styles import DARK_BLUE, ACCENT_GREEN, ACCENT_ORANGE, TEXT_WHITE, ACCENT_BLUE

class ReportWindow(QDialog):
    def __init__(self, comparison_data, resource_data, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Mission and Logistics Report - AgriSwarm")
        self.resize(900, 700)
        self.setStyleSheet(f"background-color: #ecf0f1; color: {DARK_BLUE};")
        
        layout = QVBoxLayout(self)
        layout.setSpacing(20)
        layout.setContentsMargins(30,30,30,30)
        
        # --- HEADER ---
        lbl_title = QLabel("EFFICIENCY ANALYSIS: MOBILE VS STATIC STATION")
        lbl_title.setStyleSheet(f"font-size: 22px; font-weight: bold; color: {DARK_BLUE}; margin-bottom: 10px;")
        lbl_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(lbl_title)
        
        # --- COMPARISON CARDS ---
        self.create_comparison_cards(layout, comparison_data)
        
        # --- LOGISTICS SECTION ---
        lbl_log = QLabel("FIELD RESOURCE PLANNING")
        lbl_log.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {DARK_BLUE}; margin-top: 20px;")
        layout.addWidget(lbl_log)
        
        self.create_resource_section(layout, resource_data)
        
        # --- STOPS TABLE ---
        self.create_stops_table(layout, resource_data.get('stops', []))
        
        # --- FOOTER / EXPORT ---
        btn_close = QPushButton("Close")
        btn_close.clicked.connect(self.accept)
        btn_close.setStyleSheet(f"""
            QPushButton {{
                background-color: {ACCENT_ORANGE}; color: white; padding: 10px 20px; border-radius: 5px; font-weight: bold;
            }}
            QPushButton:hover {{ background-color: #d35400; }}
        """)
        layout.addWidget(btn_close, alignment=Qt.AlignmentFlag.AlignRight)

    def create_comparison_cards(self, parent_layout, data):
        container = QHBoxLayout()
        container.setSpacing(15)
        
        # Card 1: Distance Savings
        dead_km = data.get('savings_km', 0)
        card1 = self.create_metric_card("DEADHEAD DISTANCE SAVINGS", f"{dead_km:.2f} km", "Less empty runs", positive=True)
        container.addWidget(card1)
        
        # Card 2: Efficiency
        eff = data.get('efficiency_gain_pct', 0)
        card2 = self.create_metric_card("EFFICIENCY IMPROVEMENT", f"+{eff:.1f}%", "Extra productive time", positive=True)
        container.addWidget(card2)
        
        # Card 3: Deadhead Comparison
        static_km = data.get('static_dead_km', 0)
        mobile_km = data.get('mobile_dead_km', 0)
        txt = f"Static: {static_km:.1f} km\nMobile: {mobile_km:.1f} km"
        card3 = self.create_metric_card("NON-PRODUCTIVE TRAVEL", txt, "Direct comparison", positive=None)
        container.addWidget(card3)

        parent_layout.addLayout(container)

    def create_metric_card(self, title, value, subtitle, positive=True):
        card = QFrame()
        card.setStyleSheet(f"""
            QFrame {{
                background-color: white;
                border-radius: 8px;
                border: 1px solid #bdc3c7;
            }}
        """)
        clayout = QVBoxLayout(card)
        
        lbl_t = QLabel(title)
        lbl_t.setStyleSheet("color: #7f8c8d; font-size: 11px; font-weight: bold;")
        
        lbl_v = QLabel(value)
        color = ACCENT_GREEN if positive else (ACCENT_ORANGE if positive is None else "#c0392b")
        if positive is None: color = DARK_BLUE
        
        lbl_v.setStyleSheet(f"color: {color}; font-size: 28px; font-weight: 900;")
        
        lbl_s = QLabel(subtitle)
        lbl_s.setStyleSheet("color: #95a5a6; font-size: 11px; font-style: italic;")
        
        clayout.addWidget(lbl_t)
        clayout.addWidget(lbl_v)
        clayout.addWidget(lbl_s)
        return card

    def create_resource_section(self, parent_layout, data):
        box = QFrame()
        box.setStyleSheet(f"""
            QFrame {{ background-color: {DARK_BLUE}; border-radius: 6px; color: white; }}
            QLabel {{ color: white; }}
        """)
        bl = QHBoxLayout(box)
        bl.setContentsMargins(20, 15, 20, 15)
        
        # Batteries
        packs = data.get('battery_packs', 0)
        l1 = QVBoxLayout()
        l1.addWidget(QLabel("PHYSICAL BATTERIES REQUIRED"))
        lb_p = QLabel(f"{packs} Packs")
        lb_p.setStyleSheet(f"font-size: 24px; font-weight: bold; color: {ACCENT_ORANGE};")
        l1.addWidget(lb_p)
        l1.addWidget(QLabel("(Continuous Charge Rotation)"))
        
        # Water
        mix = data.get('total_mix_l', 0)
        l2 = QVBoxLayout()
        l2.addWidget(QLabel("TOTAL WATER REQUIREMENT"))
        lb_w = QLabel(f"{mix:.1f} Liters")
        lb_w.setStyleSheet(f"font-size: 24px; font-weight: bold; color: {ACCENT_BLUE};")
        l2.addWidget(lb_w)
        l2.addWidget(QLabel("(Ready-to-apply mix)"))
        
        bl.addLayout(l1)
        
        # Vertical Separator
        line = QFrame()
        line.setFrameShape(QFrame.Shape.VLine)
        line.setFrameShadow(QFrame.Shadow.Plain)
        line.setStyleSheet("background-color: #bdc3c7;")
        bl.addWidget(line)
        
        bl.addLayout(l2)
        
        parent_layout.addWidget(box)

    def create_stops_table(self, parent_layout, stops):
        table = QTableWidget()
        table.setColumnCount(3)
        table.setHorizontalHeaderLabels(["STOP / MODULE", "LOGISTICS ACTION", "NOTES"])
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        table.verticalHeader().setVisible(False)
        table.setStyleSheet(f"""
            QTableWidget {{
                background-color: white;
                gridline-color: #ecf0f1;
                border: 1px solid #bdc3c7;
            }}
            QHeaderView::section {{
                background-color: {DARK_BLUE};
                color: white;
                padding: 5px;
                border: none;
            }}
        """)
        
        table.setRowCount(len(stops))
        for i, stop in enumerate(stops):
            table.setItem(i, 0, QTableWidgetItem(stop['name']))
            table.setItem(i, 1, QTableWidgetItem(stop['action']))
            table.setItem(i, 2, QTableWidgetItem(stop['notes']))
            
        parent_layout.addWidget(table)