# src/mission_exporter.py
import json

class QGCPlanExporter:
    """
    Exporta rutas al formato estándar .plan de QGroundControl/PX4.
    """

    @staticmethod
    def save_plan(filename, waypoints_latlon):
        """
        :param filename: Nombre del archivo .plan
        :param waypoints_latlon: Lista de tuplas [(lat, lon, alt), ...]
        """
        mission_items = []
        
        # 1. Configurar el primer item (Despegue o Dummy)
        # Por simplicidad, asumimos que la lista son waypoints de vuelo
        for i, (lat, lon, alt) in enumerate(waypoints_latlon):
            item = {
                "autoContinue": True,
                "command": 16,  # MAV_CMD_NAV_WAYPOINT
                "doJumpId": i + 1,
                "frame": 3,     # MAV_FRAME_GLOBAL_RELATIVE_ALT
                "params": [0, 0, 0, 0, lat, lon, alt],
                "type": "SimpleItem"
            }
            mission_items.append(item)

        # 2. Estructura JSON completa requerida por QGC
        plan_dict = {
            "fileType": "Plan",
            "geoFence": {"circles": [], "polygons": [], "version": 2},
            "groundStation": "AgriSwarm Planner",
            "mission": {
                "cruiseSpeed": 5, # m/s
                "firmwareType": 12, # PX4
                "hoverSpeed": 0,
                "items": mission_items,
                "plannedHomePosition": [waypoints_latlon[0][0], waypoints_latlon[0][1], 0],
                "vehicleType": 2, # Multirotor
                "version": 2
            },
            "rallyPoints": {"points": [], "version": 2},
            "version": 1
        }

        # 3. Guardar archivo
        with open(filename, 'w') as f:
            json.dump(plan_dict, f, indent=4)
        print(f"✅ Misión exportada exitosamente: {filename}")