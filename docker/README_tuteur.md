# CARLA 0.9.16 + Piste UPHF (Gyrovia) — Docker autonome

Image Docker tout-en-un : simulateur CARLA + piste UPHF integree + client Python.
Aucune installation requise : charger l'image, la lancer, la piste est la.

## 1. Charger l'image (une fois)
    docker load < carla-uphf.tar.gz
    docker images | grep carla-uphf

## 2. Mode graphique (Linux + GPU NVIDIA)
    xhost +local:docker
    docker run --rm --runtime=nvidia --net=host \
      --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all \
      --env=DISPLAY=$DISPLAY --env=RENDER_MODE=display \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      carla-uphf:0.9.16
Navigation : clic gauche + souris pour regarder, W A S D pour se deplacer.

## 3. Mode offscreen (sans GPU / Windows / Mac)
    docker run --rm --net=host --env=RENDER_MODE=offscreen carla-uphf:0.9.16

## 4. Spawn / piloter des vehicules
    CONTAINER=$(docker ps -q --filter ancestor=carla-uphf:0.9.16)
    docker exec -it $CONTAINER /opt/python310/bin/python3.10 -c "
    import carla, time
    client = carla.Client('localhost', 2000); client.set_timeout(60.0)
    world = client.get_world()
    bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    sp = world.get_map().get_spawn_points()[0]
    v = world.spawn_actor(bp, sp); v.set_autopilot(True, 8000)
    print('Vehicule en autopilot'); time.sleep(30); v.destroy()
    "

## 5. Script personnel
    docker cp mon_script.py $CONTAINER:/tmp/mon_script.py
    docker exec -it $CONTAINER /opt/python310/bin/python3.10 /tmp/mon_script.py

## Notes techniques
- Simulateur CARLA 0.9.16, client Python 3.10 (/opt/python310).
- Piste UPHF Gyrovia : xodr aligne a 100% sur le mesh (183 segments, 2 impasses).
- Map par defaut configuree dans DefaultEngine.ini (demarrage direct).
- Traffic Manager port 8000. Cache .bin volontairement absent (reconstruit au 1er chargement).
- Ports : RPC 2000-2002, TM 8000.

## Depannage
- Fenetre noire : verifier xhost +local:docker et nvidia-smi. Sinon mode offscreen.
- Port 2000 occupe : docker stop $(docker ps -q --filter ancestor=carla-uphf:0.9.16)
- "destroyed actor" en autopilot : le vehicule a atteint un bord ouvert de la piste.
