#!/usr/bin/env python3

import math
import os
import requests
import time

CENTER_LAT = 24.1467834
CENTER_LON = 47.2700878

RADIUS_METERS = 1000

ZOOMS = [15, 16, 17, 18, 19]

OUT_DIR = "offline_tiles"

# ArcGIS World Imagery
# URL order is z/y/x
TILE_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"


def latlon_to_tile(lat, lon, zoom):
    lat_rad = math.radians(lat)
    n = 2.0 ** zoom

    x = int((lon + 180.0) / 360.0 * n)

    y = int(
        (1.0 - math.asinh(math.tan(lat_rad)) / math.pi)
        / 2.0
        * n
    )

    return x, y


def meters_to_lat_delta(meters):
    return meters / 111320.0


def meters_to_lon_delta(meters, lat):
    return meters / (111320.0 * math.cos(math.radians(lat)))


def is_bad_tile(data):
    text = data[:500].lower()
    return (
        b"access blocked" in text or
        b"tile usage policy" in text or
        b"openstreetmap" in text or
        b"<html" in text
    )


def download_tile(z, x, y):
    folder = os.path.join(OUT_DIR, str(z), str(x))
    os.makedirs(folder, exist_ok=True)

    path = os.path.join(folder, f"{y}.png")

    if os.path.exists(path):
        print("Exists:", path)
        return

    url = TILE_URL.format(z=z, x=x, y=y)

    print("Downloading:", url)

    headers = {
        "User-Agent": "DroneGCSOfflineTiles/1.0"
    }

    try:
        r = requests.get(url, headers=headers, timeout=20)

        if r.status_code != 200:
            print("Failed:", r.status_code, url)
            return

        if is_bad_tile(r.content):
            print("Bad tile skipped:", url)
            return

        with open(path, "wb") as f:
            f.write(r.content)

        print("Saved:", path)

        time.sleep(0.05)

    except Exception as e:
        print("Error:", e, url)


def main():
    lat_delta = meters_to_lat_delta(RADIUS_METERS)
    lon_delta = meters_to_lon_delta(RADIUS_METERS, CENTER_LAT)

    min_lat = CENTER_LAT - lat_delta
    max_lat = CENTER_LAT + lat_delta
    min_lon = CENTER_LON - lon_delta
    max_lon = CENTER_LON + lon_delta

    print("Downloading offline tiles")
    print("Center:", CENTER_LAT, CENTER_LON)
    print("Radius:", RADIUS_METERS, "meters")
    print("Zooms:", ZOOMS)

    for z in ZOOMS:
        x1, y1 = latlon_to_tile(max_lat, min_lon, z)
        x2, y2 = latlon_to_tile(min_lat, max_lon, z)

        min_x = min(x1, x2)
        max_x = max(x1, x2)
        min_y = min(y1, y2)
        max_y = max(y1, y2)

        total = (max_x - min_x + 1) * (max_y - min_y + 1)

        print("")
        print("Zoom", z)
        print("X:", min_x, "to", max_x)
        print("Y:", min_y, "to", max_y)
        print("Tiles:", total)

        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                download_tile(z, x, y)

    print("")
    print("Done.")
    print("Generated folder:", OUT_DIR)


if __name__ == "__main__":
    main()