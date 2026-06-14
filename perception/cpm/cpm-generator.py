"""
cpm_generator.py
Les 3 véhicules CARLA détectent les autres véhicules ET les piétons
via l'API CARLA et publient des CPMs ETSI TS 103 324 conformes.

Topics : /its/cpm/v1000000001 / v1000000002 / v1000000003

Objets détectés :
  - Véhicules (type CARLA : vehicle.*) → TrafficParticipantType = passengerCar
  - Piétons   (type CARLA : walker.*) → TrafficParticipantType = pedestrian
"""
import carla
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from etsi_its_cpm_ts_msgs.msg import (
    CollectivePerceptionMessage,
    WrappedCpmContainer,
    PerceivedObject,
    Velocity3dWithConfidence,
)

CPM_HZ          = 10.0
DETECTION_RANGE = 100.0
MIN_RANGE       = 1.0
REF_LAT         = 50.31841
REF_LON         = 3.51045

VEHICLES_CONFIG = [
    (1, 1000000001),
    (2, 1000000002),
    (3, 1000000003),
]


def carla_to_gps(x, y):
    R = 6371000.0
    lat = REF_LAT - math.degrees(y / R)
    lon = REF_LON + math.degrees(x / (R * math.cos(math.radians(REF_LAT))))
    return lat, lon


def get_relative_position(det_x, det_y, det_yaw_rad, obj_x, obj_y):
    dx = obj_x - det_x
    dy = obj_y - det_y
    x_rel =  dx * math.cos(det_yaw_rad) + dy * math.sin(det_yaw_rad)
    y_rel = -dx * math.sin(det_yaw_rad) + dy * math.cos(det_yaw_rad)
    return x_rel, y_rel


def get_relative_velocity(det_vx, det_vy, det_yaw_rad, obj_vx, obj_vy):
    dvx = obj_vx - det_vx
    dvy = obj_vy - det_vy
    vx_rel =  dvx * math.cos(det_yaw_rad) + dvy * math.sin(det_yaw_rad)
    vy_rel = -dvx * math.sin(det_yaw_rad) + dvy * math.cos(det_yaw_rad)
    return vx_rel, vy_rel


class DetectorAgent:
    def __init__(self, node, idx, station_id):
        self.node       = node
        self.idx        = idx
        self.station_id = station_id
        self.actor      = None

        self.cpm_pub = node.create_publisher(
            CollectivePerceptionMessage,
            f'/its/cpm/v{station_id}', 10)

        node.get_logger().info(
            f'Détecteur {idx} (station_id={station_id}) '
            f'→ /its/cpm/v{station_id}')

    def find_actor(self, all_vehicles):
        if self.actor and self.actor.is_alive:
            return True
        alive = [v for v in all_vehicles if v.is_alive]
        if len(alive) >= self.idx:
            self.actor = alive[self.idx - 1]
            self.node.get_logger().info(
                f'Détecteur {self.idx} → carla_id={self.actor.id}')
            return True
        return False

    def detect_and_publish(self, all_vehicles, all_walkers):
        if not self.actor or not self.actor.is_alive:
            return
        try:
            det_loc     = self.actor.get_location()
            det_vel     = self.actor.get_velocity()
            det_trans   = self.actor.get_transform()
            det_yaw     = det_trans.rotation.yaw
            det_yaw_rad = math.radians(det_yaw)

            det_lat, det_lon = carla_to_gps(det_loc.x, det_loc.y)

            cpm = CollectivePerceptionMessage()
            cpm.header.protocol_version.value = 2
            cpm.header.message_id.value       = 14  # CPM
            cpm.header.station_id.value       = self.station_id

            cpm.payload.management_container \
                .reference_position.latitude.value  = int(det_lat * 1e7)
            cpm.payload.management_container \
                .reference_position.longitude.value = int(det_lon * 1e7)

            perceived_list = []
            obj_id = 1

            # ── Détecter les véhicules ───────────────────────────────────────
            for v in all_vehicles:
                if v.id == self.actor.id:
                    continue
                if not v.is_alive:
                    continue

                obj_loc = v.get_location()
                obj_vel = v.get_velocity()
                dist    = obj_loc.distance(det_loc)

                if dist < MIN_RANGE or dist > DETECTION_RANGE:
                    continue

                obj = self._build_perceived_object(
                    obj_id, obj_loc, obj_vel,
                    det_loc, det_vel, det_yaw_rad,
                    v.get_transform().rotation.yaw,
                    is_pedestrian=False)
                perceived_list.append(obj)
                obj_id += 1

            # ── Détecter les piétons ─────────────────────────────────────────
            for w in all_walkers:
                if not w.is_alive:
                    continue

                obj_loc = w.get_location()
                obj_vel = w.get_velocity()
                dist    = obj_loc.distance(det_loc)

                if dist < MIN_RANGE or dist > DETECTION_RANGE:
                    continue

                obj = self._build_perceived_object(
                    obj_id, obj_loc, obj_vel,
                    det_loc, det_vel, det_yaw_rad,
                    w.get_transform().rotation.yaw,
                    is_pedestrian=True)
                perceived_list.append(obj)

                self.node.get_logger().info(
                    f'[v{self.station_id}] 🚶 Piéton détecté '
                    f'à {dist:.1f}m | carla_id={w.id}')
                obj_id += 1

            # ── WrappedCpmContainer ──────────────────────────────────────────
            wrapped = WrappedCpmContainer()
            wrapped.container_id.value = \
                WrappedCpmContainer.CHOICE_CONTAINER_DATA_PERCEIVED_OBJECT_CONTAINER
            wrapped.container_data_perceived_object_container \
                .perceived_objects.array = perceived_list

            cpm.payload.cpm_containers.value.array = [wrapped]
            self.cpm_pub.publish(cpm)

            if perceived_list:
                n_veh = sum(1 for o in perceived_list)
                self.node.get_logger().info(
                    f'[v{self.station_id}] CPM: {n_veh} objet(s) '
                    f'| pos=({det_loc.x:.1f},{det_loc.y:.1f})')

        except Exception as e:
            self.node.get_logger().error(
                f'Détecteur {self.idx} error: {e}')

    def _build_perceived_object(self, obj_id, obj_loc, obj_vel,
                                 det_loc, det_vel, det_yaw_rad,
                                 obj_yaw_deg, is_pedestrian):
        x_rel, y_rel = get_relative_position(
            det_loc.x, det_loc.y, det_yaw_rad,
            obj_loc.x, obj_loc.y)

        vx_rel, vy_rel = get_relative_velocity(
            det_vel.x, det_vel.y, det_yaw_rad,
            obj_vel.x, obj_vel.y)

        x_cm   = max(-131072, min(131071, int(round(x_rel  * 100.0))))
        y_cm   = max(-131072, min(131071, int(round(y_rel  * 100.0))))
        vx_cms = max(-16383,  min(16382,  int(round(vx_rel * 100.0))))
        vy_cms = max(-16383,  min(16382,  int(round(vy_rel * 100.0))))

        obj = PerceivedObject()
        obj.object_id.value          = obj_id
        obj.object_id_is_present     = True
        obj.measurement_delta_time.value = 0

        # Position relative (cm)
        obj.position.x_coordinate.value.value = x_cm
        obj.position.y_coordinate.value.value = y_cm

        # Vitesse cartésienne (cm/s)
        obj.velocity_is_present = True
        obj.velocity.choice = \
            Velocity3dWithConfidence.CHOICE_CARTESIAN_VELOCITY
        obj.velocity.cartesian_velocity.x_velocity.value.value = vx_cms
        obj.velocity.cartesian_velocity.y_velocity.value.value = vy_cms

        return obj


class MultiCpmGenerator(Node):
    def __init__(self):
        super().__init__('multi_cpm_generator')

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(60.0)
        self.world  = self.client.get_world()

        self.cb_group = ReentrantCallbackGroup()
        self.agents   = []

        for idx, station_id in VEHICLES_CONFIG:
            self.agents.append(DetectorAgent(self, idx, station_id))

        self.create_timer(
            1.0 / CPM_HZ, self._tick,
            callback_group=self.cb_group)

        self.get_logger().info(
            f'Multi CPM Generator — {len(self.agents)} détecteurs / '
            f'{CPM_HZ}Hz / rayon={DETECTION_RANGE}m')

    def _tick(self):
        try:
            all_actors   = self.world.get_actors()
            all_vehicles = list(all_actors.filter('vehicle.*'))
            all_walkers  = list(all_actors.filter('walker.pedestrian.*'))

            for agent in self.agents:
                agent.find_actor(all_vehicles)

            for agent in self.agents:
                agent.detect_and_publish(all_vehicles, all_walkers)

        except Exception as e:
            self.get_logger().error(f'Tick error: {e}')


def main():
    rclpy.init()
    node = MultiCpmGenerator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
