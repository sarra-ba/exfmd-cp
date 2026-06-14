"""
main.py — ExFMD-CP Multi-Agent
SUMO contrôle les routes, CARLA fournit la physique, VehicleAgent publie les topics ROS2
"""
import argparse
import logging
import signal
import sys
import threading
import time
import os

import carla
import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml

sys.path.insert(0, '/home/syfra/CarlaSumoArtery-CoSimulation/carla/Co-Simulation/Sumo')
from sumo_integration.carla_simulation import CarlaSimulation
from sumo_integration.sumo_simulation import SumoSimulation
from run_synchronization import SimulationSynchronization

from carla_multi_agent.vehicle_agent import VehicleAgent

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("main")

SUMO_HERO_IDS = ['hero', 'hero1', 'hero2']
SUMO_CFG = os.path.expanduser(
    '/home/syfra/CarlaSumoArtery-CoSimulation/carla/Co-Simulation/Sumo/examples/Town04.sumocfg'
)

def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/simulation.yaml")
    args = parser.parse_args()
    cfg = load_config(args.config)

    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=len(SUMO_HERO_IDS) + 2)

 
    # ── Co-simulation SUMO ────────────────────────────────────────────────
    logger.info("Démarrage co-simulation SUMO...")
    sumo_sim  = SumoSimulation(SUMO_CFG, cfg["carla"]["fixed_delta_seconds"],
                                host=None, port=None, sumo_gui=False, client_order=1)
    carla_sim = CarlaSimulation(cfg["carla"]["host"], cfg["carla"]["port"],
                                cfg["carla"]["fixed_delta_seconds"])
    sync = SimulationSynchronization(sumo_sim, carla_sim,
                                     tls_manager='none',
                                     sync_vehicle_color=False,
                                     sync_vehicle_lights=False)

    agents = []

    def cleanup(signum=None, frame=None):
        logger.info("Arrêt en cours...")
        for agent in agents:
            try:
                agent.destroy_agent()
            except Exception as e:
                logger.warning("Erreur destruction: %s", e)
        try:
            sync.close()
        except Exception:
            pass
        try:
            s = world.get_settings()
            s.synchronous_mode = False
            s.fixed_delta_seconds = None
            world.apply_settings(s)
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        logger.info("Simulation terminée.")
        sys.exit(0)

    signal.signal(signal.SIGINT,  cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        # ── Attend que les heroes SUMO apparaissent dans CARLA ─────────────
        logger.info("Attente des véhicules SUMO...")
        for _ in range(100):
            sync.tick()
            world.tick()
            heroes = {k: v for k, v in sync.sumo2carla_ids.items()
                      if k in SUMO_HERO_IDS}
            if len(heroes) >= len(SUMO_HERO_IDS):
                break
            time.sleep(0.1)

        logger.info("Heroes SUMO dans CARLA: %s", heroes)

        # ── Attache VehicleAgent à chaque hero ─────────────────────────────
        for i, sumo_id in enumerate(SUMO_HERO_IDS):
            carla_id = heroes.get(sumo_id)
            if carla_id is None:
                logger.warning("Hero %s non trouvé", sumo_id)
                continue
            vehicle = world.get_actor(carla_id)
            if vehicle is None:
                logger.warning("Acteur CARLA %d non trouvé", carla_id)
                continue
            agent = VehicleAgent(vehicle, i, world, cfg)
            agents.append(agent)
            executor.add_node(agent)
            logger.info("VehicleAgent[%d] → %s (carla_id=%d)", i, sumo_id, carla_id)

        world.tick()

        logger.info("=" * 55)
        logger.info("%d véhicules actifs — topics ROS2 :", len(agents))
        for i in range(1, len(agents) + 1):
            logger.info("  /vehicle_%d/gnss/fix", i)
            logger.info("  /vehicle_%d/imu/data", i)
            logger.info("  /vehicle_%d/vehicle_status", i)
        logger.info("=" * 55)
        logger.info("Ctrl+C pour arrêter")

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # ── Boucle principale ──────────────────────────────────────────────
        hero_actor = world.get_actor(heroes.get("hero", -1))
        spec = world.get_spectator()
        prev_transforms = {agent: agent.vehicle.get_transform() for agent in agents}
        dt = cfg["carla"]["fixed_delta_seconds"]

        tick_count = 0
        ros_publish_every = max(1, int(1.0 / (cfg["carla"]["fixed_delta_seconds"] * 20)))
        while True:
            sync.tick()
            world.tick()
            tick_count += 1
            if tick_count % 20 == 0:  # toutes les ~2.5s
                all_actors = world.get_actors()
                hero_loc = hero_actor.get_location() if hero_actor and hero_actor.is_alive else None
                if hero_loc:
                    import math
                    nearby = []
                    for a in all_actors:
                        if 'vehicle' in a.type_id:
                            loc = a.get_location()
                            dist = math.sqrt((loc.x-hero_loc.x)**2+(loc.y-hero_loc.y)**2)
                            if dist < 55 and dist > 1:
                                nearby.append((dist, a.type_id))
                    nearby.sort()
                    logger.info("GT véhicules proches hero (<55m): %d", len(nearby))
                    for d, t in nearby[:8]:
                        logger.info("  %.1fm — %s", d, t)
            if hero_actor and hero_actor.is_alive:
                t = hero_actor.get_transform()
                spec.set_transform(carla.Transform(
                    carla.Location(x=t.location.x, y=t.location.y, z=t.location.z+80),
                    carla.Rotation(pitch=-90)
                ))
            if tick_count % ros_publish_every == 0:
             for agent in agents:
                if agent.vehicle.is_alive:
                    curr_t = agent.vehicle.get_transform()
                    prev_t = prev_transforms.get(agent, curr_t)
                    dx = curr_t.location.x - prev_t.location.x
                    dy = curr_t.location.y - prev_t.location.y
                    dz = curr_t.location.z - prev_t.location.z
                    import carla as _carla
                    computed_vel = _carla.Vector3D(dx/dt, dy/dt, dz/dt)
                    prev_transforms[agent] = curr_t
                    agent.update_state(
                        curr_t,
                        computed_vel,
                        agent.vehicle.get_angular_velocity(),
                        agent.vehicle.get_control(),
                    )

    except Exception as e:
        logger.exception("Erreur fatale: %s", e)
        cleanup()


if __name__ == "__main__":
    main()
