import sys
import os
from PyQt6.QtWidgets import QApplication

# Configurar path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from gui.app_window import AgriSwarmApp
# Importar datos para inicializar DB
import drone_data 

def main():
    # Asegurar directorios
    base_dir = os.path.dirname(__file__)
    data_dir = os.path.join(base_dir, 'data')
    if not os.path.exists(data_dir): os.makedirs(data_dir)
    filename = os.path.join(data_dir, 'mi_campo_dibujado.json')

    # Iniciar App Qt
    app = QApplication(sys.argv)
    
    # Crear ventana principal
    window = AgriSwarmApp(filename=filename)
    window.show() # Mostrar ventana

    # Bucle de eventos
    sys.exit(app.exec())

if __name__ == "__main__":
    main()