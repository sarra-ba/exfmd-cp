import random
import logging
import carla

logger = logging.getLogger(__name__)

class TrafficManagerSetup:
    def __init__(self, client, world, cfg):
        self.client = client
        self.world  = world
        self.cfg    = cfg
        self.npc_actors = []
        self.tm = client.get_trafficmanager(8000)
        self.tm.set_synchronous_mode(True)
        self.tm.set_global_distance_to_leading_vehicle(2.5)
        self.tm.set_respawn_dormant_vehicles(True)
        if cfg["traffic"]["hybrid_physics"]:
            self.tm.set_hybrid_physics_mode(True)
            self.tm.set_hybrid_physics_radius(70.0)

    def configure_ego_vehicle(self, vehicle, vehicle_index):
        vehicle.set_autopilot(True, self.tm.get_port())
        speeds = [0.0, -10.0, 10.0]
        self.tm.vehicle_percentage_speed_difference(vehicle, speeds[vehicle_index % 3])
        self.tm.distance_to_leading_vehicle(vehicle, 3.0)
        self.tm.auto_lane_change(vehicle, True)
        self.tm.ignore_lights_percentage(vehicle, 0.0)

    def spawn_npc_traffic(self):
        bp_lib = self.world.get_blueprint_library()
        spawn_pts = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_pts)
        vehicle_bps = [bp for bp in bp_lib.filter("vehicle.*") if int(bp.get_attribute("number_of_wheels")) == 4]
        n_vehicles = min(self.cfg["traffic"]["npc_vehicles"], len(spawn_pts))
        batch = []
        for i in range(n_vehicles):
            bp = random.choice(vehicle_bps)
            if bp.has_attribute("color"):
                bp.set_attribute("color", random.choice(bp.get_attribute("color").recommended_values))
            batch.append(carla.command.SpawnActor(bp, spawn_pts[i])
                         .then(carla.command.SetAutopilot(carla.command.FutureActor, True, self.tm.get_port())))
        results = self.client.apply_batch_sync(batch, False)
        spawned = 0
        for r in results:
            if not r.error:
                self.npc_actors.append(self.world.get_actor(r.actor_id))
                spawned += 1
        logger.info("Trafic NPC : %d/%d véhicules spawnés", spawned, n_vehicles)

    def destroy(self):
        if self.npc_actors:
            self.client.apply_batch_sync([carla.command.DestroyActor(a) for a in self.npc_actors], True)
            logger.info("%d acteurs NPC détruits", len(self.npc_actors))
            self.npc_actors.clear()
