# V2X Apps LDM Dashboard Backend

This package now includes a real-time LDM backend in `v2x_apps/ldm_server.py` for C-ITS dashboard use cases.

## LDM data sources

The LDM server subscribes to:

- `/its/cam_received` (CAM) → updates `stations`
- `ws://<central-server>/ws/cam` (optional central CAM websocket) → updates `stations`
- `/its/cpm` (CPM) → updates `perceived_objects`
- `/parking/poim_outgoing` (JSON String) → updates `pois`
- `/parking/poim_incoming` (JSON String) → updates `pois`
- `/parking/poim_decoded` (JSON String) → updates `pois`

## LDM state model

The server maintains an internal in-memory store:

- `stations`: keyed by CAM station id
- `perceived_objects`: keyed by `{source_station_id}:{object_id}`
- `pois`: keyed by POI id

Each record stores `last_update` internally. Stale entries older than 5 seconds are removed automatically.

## HTTP + WebSocket API

- `GET /ldm/state` → full live LDM snapshot (`stations`, `perceived_objects`, `pois`)
- `GET /ldm/geojson` → GeoJSON projection of the same LDM data
- `WS /ws/ldm` → pushes updated LDM snapshots whenever CAM/CPM/POIM updates are processed or stale entries are pruned

## Central CAM feed integration

`ldm_server` can subscribe directly to the external central CAM websocket and merge received CAMs into the same `stations` collection served to the dashboard.

Run with central feed enabled:

```bash
ros2 run v2x_apps ldm_server --ros-args \
  -p central_cam_enabled:=true \
  -p central_cam_ws_url:=ws://127.0.0.1:8010/ws/cam
```

When central mode is enabled, the node keeps reconnecting automatically if the central server is temporarily unavailable.

## Expose central server to internet

Start the central server on all interfaces:

```bash
python /tmp/workspace/Aniisss/c-its/central_server.py --host 0.0.0.0 --port 8010
```

If it is published behind a reverse proxy, enable forwarded headers:

```bash
python /tmp/workspace/Aniisss/c-its/central_server.py \
  --host 0.0.0.0 --port 8010 \
  --proxy-headers --forwarded-allow-ips='*'
```

Then point the simulator node to the public websocket:

```bash
ros2 run v2x_apps cam_provider_simulator_centralServer --ros-args \
  -p central_ws_ingest_url:=wss://<your-public-domain>/ws/ingest
```

## ETSI station type support

`stations[].station_type` is exposed from CAM basic container data and can be used in frontend icon mapping logic, e.g.:

- Type `5`: passenger car
- Type `15`: RSU

Use this value in the dashboard map layer to choose station icons.
