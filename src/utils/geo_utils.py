import math
import json

class GeoUtils:
    """
    Utilidades para conversión de coordenadas planas a geodésicas (WGS84)
    y exportación de misiones.
    Referencia por defecto: Santa Cruz, Bolivia.
    """
    
    # Constantes WGS84
    R_EARTH = 6378137.0

    @staticmethod
    def enu_to_geodetic(x, y, z, lat0, lon0, alt0):
        """
        Convierte coordenadas locales ENU (East-North-Up) a GPS (Lat, Lon, Alt).
        """
        rad_lat0 = math.radians(lat0)
        
        # Diferencias en metros a grados
        d_lat = y / GeoUtils.R_EARTH
        d_lon = x / (GeoUtils.R_EARTH * math.cos(rad_lat0))

        new_lat = lat0 + math.degrees(d_lat)
        new_lon = lon0 + math.degrees(d_lon)
        new_alt = alt0 + z

        return new_lat, new_lon, new_alt

    @staticmethod
    def export_qgc_mission(waypoints, filename, home_lat=-17.3935, home_lon=-63.2622):
        """
        Genera un archivo .plan compatible con QGroundControl / PX4.
        :param waypoints: Lista de tuplas (x, y) en metros.
        :param filename: Nombre del archivo de salida.
        :param home_lat: Latitud del punto (0,0) local.
        :param home_lon: Longitud del punto (0,0) local.
        """
        mission_items = []
        altitude = 10.0 # Metros de altura de vuelo sobre el suelo
        
        # 1. Waypoint inicial (Despegue / Takeoff)
        mission_items.append(GeoUtils._create_mission_item(0, home_lat, home_lon, 0, "TAKEOFF"))

        # 2. Ruta de fumigación
        for i, (x, y) in enumerate(waypoints):
            # Convertir cada punto (x,y) metros -> Lat/Lon global
            lat, lon, alt = GeoUtils.enu_to_geodetic(x, y, altitude, home_lat, home_lon, 0)
            mission_items.append(GeoUtils._create_mission_item(i+1, lat, lon, alt, "WAYPOINT"))

        # 3. Estructura JSON de QGroundControl
        plan = {
            "fileType": "Plan",
            "geoFence": {"circles": [], "polygons": [], "version": 2},
            "groundStation": "AgriSwarm Planner",
            "mission": {
                "cruiseSpeed": 5,
                "firmwareType": 12, # PX4
                "hoverSpeed": 0,
                "items": mission_items,
                "plannedHomePosition": [home_lat, home_lon, 0],
                "vehicleType": 2, # Multirotor
                "version": 2
            },
            "rallyPoints": {"points": [], "version": 2},
            "version": 1
        }

        with open(filename, 'w') as f:
            json.dump(plan, f, indent=4)
        
        print(f"Misión exportada (Ref: Santa Cruz): {filename}")

    @staticmethod
    def _create_mission_item(seq, lat, lon, alt, type_cmd):
        """Helper para crear items de misión MAVLink"""
        # 22 = TAKEOFF, 16 = WAYPOINT
        cmd_id = 22 if type_cmd == "TAKEOFF" else 16 
        return {
            "autoContinue": True,
            "command": cmd_id,
            "doJumpId": seq,
            "frame": 3, # Global Relative Altitude (3)
            "params": [0, 0, 0, 0, lat, lon, alt],
            "type": "SimpleItem"
        }