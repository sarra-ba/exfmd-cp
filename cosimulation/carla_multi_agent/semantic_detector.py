"""
semantic_detector.py — Perception parfaite via Semantic LiDAR CARLA
Remplace PointPillars pour la phase de validation ExFMD-CP.
Même format de sortie que pointpillar_detector.py.
"""
import numpy as np
import carla
import threading
import math

# Semantic tags CARLA
VEHICLE_TAGS = {14}  # Vehicles (tag 14 = tous véhicules dans CARLA 0.9.16)
WALKER_TAGS  = {4}        # Pedestrians

CLASS_MAP = {
    10: ('Car', 1),
    14: ("Car", 1),
    4:  ('Pedestrian', 2),
}

class SemanticDetector:
    def __init__(self, world, hero_vehicle, cfg):
        self.world = world
        self.hero = hero_vehicle
        self._lock = threading.Lock()
        self._latest_objects = []
        self._sensor = None
        self._attach(cfg)

    def _attach(self, cfg):
        bp_lib = self.world.get_blueprint_library()
        bp = bp_lib.find('sensor.lidar.ray_cast_semantic')
        bp.set_attribute('channels',           str(cfg['sensors']['lidar']['channels']))
        bp.set_attribute('range',              str(cfg['sensors']['lidar']['range']))
        bp.set_attribute('points_per_second',  str(cfg['sensors']['lidar']['points_per_second']))
        bp.set_attribute('rotation_frequency', str(cfg['sensors']['lidar']['rotation_frequency']))
        bp.set_attribute('upper_fov',          '2.0')
        bp.set_attribute('lower_fov',          '-26.0')
        self._sensor = self.world.spawn_actor(
            bp,
            carla.Transform(carla.Location(z=2.5)),
            attach_to=self.hero
        )
        self._sensor.listen(self._callback)
        print(f'[SemanticDetector] Sensor spawné sur {self.hero.type_id}')

    def _callback(self, data):
        """Traitement du semantic LiDAR → bounding boxes."""
        # raw_data: array de (x, y, z, cos_inc_angle, object_idx, semantic_tag)
        pts = np.frombuffer(data.raw_data, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('cos_inc_angle', np.float32),
            ('object_idx', np.uint32),
            ('semantic_tag', np.uint32)
        ]))

        objects = []
        # Grouper par object_idx
        unique_ids = np.unique(pts['object_idx'])
        for obj_id in unique_ids:
            if obj_id == 0:
                continue  # ignorer le fond
            mask = pts['object_idx'] == obj_id
            obj_pts = pts[mask]
            tag = int(obj_pts['semantic_tag'][0])

            if tag not in VEHICLE_TAGS and tag not in WALKER_TAGS:
                continue

            # Ignorer le hero lui-même
            actor = self.world.get_actor(int(obj_id))
            if actor is None or actor.id == self.hero.id:
                continue

            xs = obj_pts['x']
            ys = obj_pts['y']
            zs = obj_pts['z']

            if len(xs) < 2:  # trop peu de points → ignorer
                continue

            cx = float(np.mean(xs))
            cy = float(np.mean(ys))
            cz = float(np.mean(zs))
            dist = math.sqrt(cx**2 + cy**2)

            if dist > 55 or dist < 0.5:
                continue

            cls_name, cls_id = CLASS_MAP.get(tag, ('Car', 1))

            objects.append({
                'x':         cx,
                'y':         cy,
                'z':         cz,
                'distance':  dist,
                'velocity':  0.0,
                'class':     cls_name,
                'class_id':  cls_id,
                'score':     1.0,  # perception parfaite
                'point_count': int(len(xs)),
                'object_id': int(obj_id),
            })

        with self._lock:
            self._latest_objects = objects

    def detect(self):
        """Retourne les derniers objets détectés."""
        with self._lock:
            return list(self._latest_objects)

    def destroy(self):
        if self._sensor and self._sensor.is_alive:
            self._sensor.stop()
            self._sensor.destroy()
