"""Normalize FTW polygon JSONs to local origin.

Takes polygon files with UTM (absolute) coordinates and translates them so
the SW corner is at (0, 0). Also re-anchors the base_point if it's outside
the polygon's local frame.

Usage:
    python tools/normalize_ftw_polygons.py \
        --src "tests/test_fields/polygons_export(1)/polygons_export" \
        --dst tests/test_fields/ftw
"""

import argparse
import json
import os
import shutil

from pyproj import Transformer


# Country → (EPSG UTM code, hemisphere letter for display only).
# Derived from typical UTM zones per country (user-provided table).
COUNTRY_EPSG = {
    "brazil":   32723,  # 23S  (MATOPIBA / oeste Bahia — where the FTW sample clusters)
    "france":   32631,  # 31N
    "rwanda":   32736,  # 36S (Rwanda lies on the 35S/36S boundary; FTW sample is in 36S)
    "vietnam":  32648,  # 48N
    "austria":  32633,  # 33N
}


def _country_from_filename(name):
    stem = name.split(".")[0].lower()
    for country in COUNTRY_EPSG:
        if stem.startswith(country):
            return country
    return None


def _utm_to_lnglat(easting, northing, epsg):
    t = Transformer.from_crs(f"EPSG:{epsg}", "EPSG:4326", always_xy=True)
    lng, lat = t.transform(easting, northing)
    return [round(lng, 7), round(lat, 7)]


def normalize_file(src_path, dst_path):
    with open(src_path) as f:
        data = json.load(f)

    b = data["boundary"]
    xs = [p[0] for p in b]
    ys = [p[1] for p in b]
    min_x, min_y = min(xs), min(ys)

    data["boundary"] = [[round(x - min_x, 2), round(y - min_y, 2)] for x, y in b]

    if "obstacles" in data and data["obstacles"]:
        data["obstacles"] = [
            [[round(x - min_x, 2), round(y - min_y, 2)] for x, y in obs]
            for obs in data["obstacles"]
        ]

    bp = data.get("base_point")
    if bp is not None:
        bx, by = bp
        if abs(bx) < 100 and abs(by) < 100:
            data["base_point"] = [round(bx, 2), round(by, 2)]
        else:
            data["base_point"] = [round(bx - min_x, 2), round(by - min_y, 2)]

    if "ugv_polyline" in data and data["ugv_polyline"]:
        data["ugv_polyline"] = [
            [round(x - min_x, 2), round(y - min_y, 2)]
            for x, y in data["ugv_polyline"]
        ]

    data["_utm_origin"] = [min_x, min_y]

    country = _country_from_filename(os.path.basename(src_path))
    if country is not None:
        epsg = COUNTRY_EPSG[country]
        data["_origin_lnglat"] = _utm_to_lnglat(min_x, min_y, epsg)
        data["_utm_epsg"] = epsg

    with open(dst_path, "w") as f:
        json.dump(data, f, indent=2)

    area_approx = _shoelace_area(data["boundary"])
    return area_approx


def _shoelace_area(boundary):
    n = len(boundary)
    s = 0.0
    for i in range(n):
        x1, y1 = boundary[i]
        x2, y2 = boundary[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    args = ap.parse_args()

    os.makedirs(args.dst, exist_ok=True)

    files = sorted(f for f in os.listdir(args.src) if f.endswith(".json"))
    print(f"Normalizing {len(files)} file(s) -> {args.dst}")
    for name in files:
        src = os.path.join(args.src, name)
        dst = os.path.join(args.dst, name)
        area_m2 = normalize_file(src, dst)
        print(f"  {name:30} area={area_m2/10000:6.2f} ha")


if __name__ == "__main__":
    main()
