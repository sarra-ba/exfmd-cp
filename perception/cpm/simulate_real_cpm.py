"""
Générateur de CPMs simulés conformes ETSI TS 103 324
Simule un véhicule réel (Origin DS7) circulant sur la piste UPHF
"""
import carla
import math
import json
import time
import os

# Connexion CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
amap = world.get_map()

# Chemin du fichier CPM reçu
CPM_OUTPUT = '/home/syfra/artery_fresh/scenarios/F2MD_Highway_CPM/carla_received_cpm.json'

def gps_to_carla(lat, lon, lat_0=50.31841, lon_0=3.51045):
    R = 6371000
    x = R * math.cos(math.radians(lat_0)) * math.radians(lon - lon_0)
    y = -(R * math.radians(lat - lat_0))
    return x, y

def carla_to_gps(x, y, lat_0=50.31841, lon_0=3.51045):
    R = 6371000
    lon = lon_0 + math.degrees(x / (R * math.cos(math.radians(lat_0))))
    lat = lat_0 - math.degrees(y / R)
    return lat, lon

# Générer waypoints GPS de la piste
waypoints = amap.generate_waypoints(5.0)
waypoints_gps = []
for wp in waypoints:
    geo = amap.transform_to_geolocation(wp.transform.location)
    waypoints_gps.append({
        'lat': geo.latitude,
        'lon': geo.longitude,
        'heading': wp.transform.rotation.yaw,
        'x': wp.transform.location.x,
        'y': wp.transform.location.y
    })

print(f"Piste UPHF: {len(waypoints_gps)} waypoints GPS générés")

# Simuler circulation du véhicule réel
station_id = 12345678  # ID fixe pour le véhicule réel simulé
speed_ms = 10.0  # 10 m/s = 36 km/h

idx = 0
print("Génération des CPMs simulés... Ctrl+C pour arrêter")
try:
    while True:
        wp = waypoints_gps[idx % len(waypoints_gps)]
        next_wp = waypoints_gps[(idx + 1) % len(waypoints_gps)]

        sender_lat = wp['lat']
        sender_lon = wp['lon']
        heading = math.radians(wp['heading'])

        # Vitesse du véhicule réel
        vx = speed_ms * math.cos(heading)
        vy = speed_ms * math.sin(heading)

        # Obtenir waypoint CARLA le plus proche pour snap sur route
        carla_x, carla_y = gps_to_carla(sender_lat, sender_lon)
        loc = carla.Location(x=carla_x, y=carla_y, z=0)
        snapped_wp = amap.get_waypoint(loc, project_to_road=True,
                                        lane_type=carla.LaneType.Driving)

        # Position corrigée
        snapped_geo = amap.transform_to_geolocation(snapped_wp.transform.location)

        # Construire CPM conforme
        cpm = {
            "sender": station_id,
            "simtime": time.time(),
            "sender_lat": snapped_geo.latitude,
            "sender_lon": snapped_geo.longitude,
            "sender_heading": snapped_wp.transform.rotation.yaw,
            "sender_speed": speed_ms,
            "objects": [],
            "count": 0
        }

        # Écrire le CPM
        with open(CPM_OUTPUT, 'w') as f:
            json.dump(cpm, f)

        print(f"CPM #{idx}: lat={snapped_geo.latitude:.6f}, lon={snapped_geo.longitude:.6f}, heading={snapped_wp.transform.rotation.yaw:.1f}°")

        idx += 1
        time.sleep(0.1)  # 10 Hz

except KeyboardInterrupt:
    print("Simulation CPM arrêtée.")
