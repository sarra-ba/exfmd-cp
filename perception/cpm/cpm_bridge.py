"""
Bridge CPM → CARLA
Lit les CPMs reçus et positionne les objets perçus sur la piste UPHF
"""
import carla
import math
import json
import time

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
amap = world.get_map()

CPM_PATH = '/home/syfra/artery_fresh/scenarios/F2MD_Highway_CPM/carla_received_cpm.json'

def gps_to_carla(lat, lon, lat_0=50.31841, lon_0=3.51045):
    R = 6371000
    x = R * math.cos(math.radians(lat_0)) * math.radians(lon - lon_0)
    y = -(R * math.radians(lat - lat_0))
    return x, y

def get_object_absolute_position(sender_lat, sender_lon, sender_heading, obj_x, obj_y):
    """
    Reconstitue la position absolue CARLA d'un objet perçu
    depuis la position GPS de l'émetteur et les distances relatives
    """
    # Position CARLA de l'émetteur
    sx, sy = gps_to_carla(sender_lat, sender_lon)

    # Convertir heading en radians
    heading_rad = math.radians(sender_heading)

    # Rotation des distances relatives selon le heading de l'émetteur
    # x = distance longitudinale (devant), y = distance latérale (gauche)
    abs_x = sx + obj_x * math.cos(heading_rad) - obj_y * math.sin(heading_rad)
    abs_y = sy + obj_x * math.sin(heading_rad) + obj_y * math.cos(heading_rad)

    return abs_x, abs_y

def validate_and_snap(x, y, obj_yaw):
    """Snappe la position sur la route et valide la direction"""
    loc = carla.Location(x=x, y=y, z=0)
    wp = amap.get_waypoint(loc, project_to_road=True,
                           lane_type=carla.LaneType.Driving)
    if wp is None:
        return None, True

    # Distance de snap
    snap_dist = math.sqrt(
        (wp.transform.location.x - x)**2 +
        (wp.transform.location.y - y)**2)

    is_suspicious = snap_dist > 10.0



    if snap_dist > 2.0:
        print(f"  ⚠ Position corrigée: {snap_dist:.1f}m de la route")

    return wp, is_suspicious

# Blueprints
bp_perceived = world.get_blueprint_library().find('vehicle.tesla.model3')
bp_perceived.set_attribute('color', '0,0,255')  # bleu = objet perçu

perceived_vehicles = {}  # {obj_id: carla_actor}
last_simtime = 0

print("CPM Bridge démarré - Ctrl+C pour arrêter")
try:
    while True:
        try:
            with open(CPM_PATH) as f:
                cpm = json.load(f)
        except:
            time.sleep(0.1)
            continue

        if cpm.get('simtime') == last_simtime:
            time.sleep(0.05)
            continue
        last_simtime = cpm.get('simtime')

        sender_lat = cpm.get('sender_lat')
        sender_lon = cpm.get('sender_lon')
        sender_heading = cpm.get('sender_heading', 0.0)

        if not sender_lat or not sender_lon:
            continue

        for obj in cpm.get('objects', []):
            obj_id = obj['id']
            obj_x = obj['x']
            obj_y = obj['y']
            obj_yaw = obj.get('yaw', sender_heading)

            # Reconstituer position absolue
            abs_x, abs_y = get_object_absolute_position(
                sender_lat, sender_lon, sender_heading, obj_x, obj_y)

            # Valider et snappe
            wp, suspicious = validate_and_snap(abs_x, abs_y, obj_yaw)

            if wp is None:
                print(f"  ✗ Objet {obj_id} hors piste → ignoré")
                continue

            status = "⚠ SUSPECT" if suspicious else "✓ OK"
            print(f"Objet {obj_id}: abs_x={abs_x:.1f}, abs_y={abs_y:.1f} → CARLA({wp.transform.location.x:.1f},{wp.transform.location.y:.1f}) {status}")

            # Spawner ou déplacer
            t = wp.transform
            t.location.z += 0.5
            t.rotation.yaw = obj_yaw

            if obj_id not in perceived_vehicles or not perceived_vehicles[obj_id].is_alive:
                t.location.z += 1.0
                try:
                    perceived_vehicles[obj_id] = world.spawn_actor(bp_perceived, t)
                    print(f"  → Objet {obj_id} spawné (id={perceived_vehicles[obj_id].id})")
                except Exception as e:
                    print(f"  → Spawn failed: {e}")
            else:
                perceived_vehicles[obj_id].set_transform(t)

        time.sleep(0.05)

except KeyboardInterrupt:
    for v in perceived_vehicles.values():
        if v.is_alive:
            v.destroy()
    print("Bridge arrêté.")
