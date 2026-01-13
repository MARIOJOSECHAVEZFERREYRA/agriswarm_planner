# src/geo_utils.py
import pymap3d as pm

class GeoTransformer:
    """
    Convierte coordenadas Cartesianas (x, y en metros) a Geodésicas (Lat, Lon).
    Usa un sistema de coordenadas Local ENU (East-North-Up).
    """

    def __init__(self, ref_lat, ref_lon, ref_alt=400.0):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt = ref_alt

    def meters_to_gps(self, x, y, z=10.0):
        """
        Convierte un punto (x, y) local a (lat, lon, alt) global.
        :param x: Metros al Este (East)
        :param y: Metros al Norte (North)
        :param z: Altura de vuelo relativa
        :return: (lat, lon, alt_amsl)
        """
        lat, lon, alt = pm.enu2geodetic(
            x, y, 0,  # Asumimos Z=0 en la proyección plana para la posición
            self.ref_lat, self.ref_lon, self.ref_alt
        )
        # Devolvemos la altitud deseada de vuelo (relativa al despegue + terreno)
        return lat, lon, self.ref_alt + z