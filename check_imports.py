import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

try:
    print("Importing styles...")
    from gui.styles import DARK_BLUE
    print("Styles OK.")

    print("Importing ReportPanel...")
    from gui.report_panel import ReportPanel
    print("ReportPanel OK.")

    print("Importing AgriSwarmApp...")
    from gui.app_window import AgriSwarmApp
    print("AgriSwarmApp OK.")

except Exception as e:
    import traceback
    traceback.print_exc()

# Test Instantiation
try:
    print("Creating QApplication...")
    from PyQt6.QtWidgets import QApplication
    app = QApplication([])
    print("QApplication OK.")
    
    print("Instantiating AgriSwarmApp...")
    from gui.app_window import AgriSwarmApp
    window = AgriSwarmApp(filename="test.json")
    print("AgriSwarmApp Instantiated OK.")
    
except Exception as e:
    import traceback
    traceback.print_exc()
