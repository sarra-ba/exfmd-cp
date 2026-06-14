# ExFMD-CP — Misbehavior Detection in Collective Perception (V2X Co-Simulation)

Framework de co-simulation V2X (CARLA + SUMO + Artery/OMNeT++) pour la
detection de comportements malveillants dans les messages de perception
collective (CPM, ETSI TS 103 324), avec integration d'un vehicule reel
(Origin DS7) sur la piste UPHF/Gyrovia.

## Architecture

Le projet couvre plusieurs scenarios complementaires :

### 1. Co-simulation (`cosimulation/`)
SUMO controle les routes, CARLA fournit la physique, des agents ROS2
publient les topics. Le module `sumo-integration/` contient l'integration
CARLA-SUMO-Artery (services Artery patches : CaService, BasicNodeManager).
- `main.py`, `main_uphf.py` : orchestration multi-agent
- `carla_multi_agent/` : agents vehicule, detecteurs (fusion LiDAR+camera,
  PointPillars, semantique), traffic manager

### 2. Perception (`perception/`)
Generation et pont des messages CAM/CPM.
- `bridges/` : ponts CAM (plusieurs variantes selon le mode de pilotage)
- `cpm/` : generation CPM ETSI TS 103 324, pont CPM->CARLA
- voir `perception/bridges/README.md` pour le detail des variantes

### 3. Vehicule reel (`real-vehicle/`)
Integration du vehicule Origin DS7 (CAM/CPM reels via le cube C-ITS).
- `replay_trajectory.py`, `fake_cam_publisher.py` : rejeu de trajectoires
  reelles (uphf_trajectory.csv)
- `gnss_to_shm.py`, `art_spawn.py`, `artery_ros_bridge.py` : ponts GNSS/Artery

### 4. Piste UPHF (`map-uphf/`)
Modelisation de la piste Gyrovia (UPHF) pour CARLA (xodr + assets).

### 5. Docker autonome (`docker/`)
Image Docker tout-en-un : CARLA + piste UPHF + client Python.
Voir `docker/README_tuteur.md`.

### Utilitaires
- `dashboards/` : tableaux de bord de visualisation
- `spawners/` : scripts de spawn de vehicules/pietons

## Pre-requis
- CARLA 0.9.16, ROS2 Humble, SUMO, Artery/OMNeT++ 5.7.1
- Voir les README de chaque sous-dossier pour les details

## Auteur
Sarra — PFE UPHF/IEMN (groupe COMNUM) x Sup'Com
Encadrants : Marwane Ayaida (UPHF/IEMN), Sameh Najeh (Sup'Com)
