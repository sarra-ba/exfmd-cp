#!/bin/bash
RENDER_MODE="${RENDER_MODE:-display}"
PY=/opt/python310/bin/python3.10
echo "=== CARLA 0.9.16 + Piste UPHF | Mode: $RENDER_MODE ==="
CARLA_ARGS="-nosound -quality-level=Epic"
if [ "$RENDER_MODE" = "offscreen" ]; then
    CARLA_ARGS="$CARLA_ARGS -RenderOffScreen"
    echo "[INFO] Mode offscreen : pas de fenetre. Port 2000."
else
    echo "[INFO] Mode display : fenetre CARLA avec la piste UPHF."
fi

cd /workspace
./CarlaUE4.sh $CARLA_ARGS &
CARLA_PID=$!

echo "[INFO] Attente initialisation CARLA..."
for i in $(seq 1 90); do
    if $PY -c "import carla; carla.Client('localhost',2000).get_server_version()" 2>/dev/null; then
        echo "[INFO] Serveur pret."; break
    fi
    sleep 1
done

# Teleporter la camera en vue de dessus sur la piste
$PY - <<PYEOF
import carla
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(60.0)
    world = client.get_world()
    m = world.get_map()
    wps = m.generate_waypoints(10.0)
    cx = sum(w.transform.location.x for w in wps)/len(wps)
    cy = sum(w.transform.location.y for w in wps)/len(wps)
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(
        carla.Location(x=cx, y=cy, z=180),
        carla.Rotation(pitch=-90)))
    print(f"[INFO] Camera en vue de dessus. Map: {m.name}")
except Exception as e:
    print("[WARN] Teleport echoue:", e)
PYEOF

echo "=== PRET. Piste UPHF chargee. Client Python: $PY ==="
echo "=== Pour spawn/piloter: docker exec -it <container> $PY votre_script.py ==="
wait $CARLA_PID
