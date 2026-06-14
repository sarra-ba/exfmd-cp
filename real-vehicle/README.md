# Vehicule reel — Origin DS7 & cube C-ITS

Integration du vehicule reel Origin DS7 (piste UPHF) dans ExFMD-CP.

## Scripts (rejeu / pont)
- replay_trajectory.py : rejoue une trajectoire reelle dans CARLA
- fake_cam_publisher.py : publie des CAMs depuis la trajectoire reelle a 10Hz
- gnss_to_shm.py : pont GNSS CARLA -> memoire partagee (/dev/shm)
- art_spawn.py : spawn vehicules + export positions vers Artery (CaService)
- artery_ros_bridge.py : pont Artery <-> ROS2

## c-its/ — Cube C-ITS embarque (DS7)
Stack ROS2 du cube C-ITS reel (CAM/CPM over ITS-G5, domaine ROS2 42).
- central_server.py : serveur central LDM
- run_cits_stack.sh : lancement de la stack
- test.py, test2.py : scripts de test
- 192.168.0-metadata.json : metadonnees reseau

## Donnees
- uphf_trajectory.csv : trajectoire reelle DS7 (2.22 km, 376s)

## Note
Rosbags, LDM dashboard et domain_bridge non versionnes (volumineux).
