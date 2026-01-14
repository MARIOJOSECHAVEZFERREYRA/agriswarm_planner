# examples/03_generate_mission.py
import sys
import os

# Agregar 'src' al path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from geometry import FieldGenerator
from divider import FieldDivider
from path_planner import CoveragePlanner
from geo_utils import GeoTransformer
from mission_exporter import QGCPlanExporter

def main():
    # --- CONFIGURACIÓN DE ESCENARIO ---
    # Coordenada real: Zona agrícola al norte de Santa Cruz (cerca de Warnes)
    HOME_LAT = -17.525654
    HOME_LON = -63.136892
    ALTURA_VUELO = 10.0 # metros
    
    # --- 1. MATEMÁTICA PURA ---
    campo = FieldGenerator.create_l_shape_field() # 200x100 metros
    sub_areas = FieldDivider.divide_vertically(campo, num_drones=3)
    planner = CoveragePlanner(sweep_width=10.0)

    # --- 2. TRANSFORMADOR GPS ---
    geo = GeoTransformer(HOME_LAT, HOME_LON)

    # --- 3. PROCESO ---
    for i, area in enumerate(sub_areas):
        if area.is_empty: continue
        
        # A. Generar ruta local (metros)
        ruta_local, _ = planner.generate_optimized_path(area)
        
        # B. Convertir a GPS (Lat/Lon)
        ruta_gps = []
        # Aplanamos la ruta (que viene en segmentos) a una lista continua de puntos
        for segmento in ruta_local:
            for punto_metro in segmento:
                lat, lon, alt = geo.meters_to_gps(punto_metro[0], punto_metro[1], ALTURA_VUELO)
                ruta_gps.append((lat, lon, alt))
        
        # C. Exportar Archivo
        nombre_archivo = f"mision_dron_{i+1}.plan"
        QGCPlanExporter.save_plan(nombre_archivo, ruta_gps)

if __name__ == "__main__":
    main()