"""
map_dashboard.py
Découvre automatiquement tous les topics /its/cam/v* et affiche
les véhicules en temps réel sur OpenStreetMap via Leaflet.
"""
import threading
import json
import subprocess
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from etsi_its_cam_msgs.msg import CAM

WEB_PORT = 8888

COLORS = [
    '#e74c3c', '#3498db', '#2ecc71', '#f39c12',
    '#9b59b6', '#1abc9c', '#e67e22', '#e91e63',
]

vehicles_state = {}
state_lock = threading.Lock()

HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ExFMD-CP — Live Map</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    body { margin: 0; padding: 0; font-family: Arial, sans-serif; background: #1a1a2e; }
    #header {
      background: #16213e; color: white; padding: 10px 20px;
      display: flex; align-items: center; justify-content: space-between;
    }
    #header h1 { margin: 0; font-size: 18px; color: #00d4ff; }
    #status { font-size: 12px; color: #aaa; }
    #map { height: calc(100vh - 50px); }
    #legend {
      position: fixed; bottom: 20px; left: 20px; z-index: 1000;
      background: rgba(22,33,62,0.9); color: white;
      padding: 10px 15px; border-radius: 8px; font-size: 12px;
      max-height: 300px; overflow-y: auto;
    }
    .legend-item { display: flex; align-items: center; margin: 4px 0; }
    .legend-dot { width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; flex-shrink: 0; }
  </style>
</head>
<body>
  <div id="header">
    <h1>🚗 ExFMD-CP — Live Vehicle Tracking</h1>
    <span id="status">Connexion...</span>
  </div>
  <div id="map"></div>
  <div id="legend">
    <div style="font-weight:bold;margin-bottom:6px;">Véhicules</div>
    <div id="legend-items"></div>
  </div>
  <script>
    const map = L.map('map').setView([50.31841, 3.51045], 17);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© OpenStreetMap contributors', maxZoom: 19
    }).addTo(map);

    const markers = {}, trails = {}, trailLines = {};
    const TRAIL_LEN = 100;
    let knownVehicles = {};

    function makeIcon(color) {
      return L.divIcon({
        className: '',
        html: `<div style="width:16px;height:16px;border-radius:50%;
               background:${color};border:2px solid white;
               box-shadow:0 0 8px ${color};"></div>`,
        iconSize: [16,16], iconAnchor: [8,8]
      });
    }

    function updateLegend(data) {
      const container = document.getElementById('legend-items');
      Object.entries(data).forEach(([vid, info]) => {
        if (!knownVehicles[vid]) {
          knownVehicles[vid] = info.color;
          const item = document.createElement('div');
          item.className = 'legend-item';
          item.id = `legend-${vid}`;
          item.innerHTML = `<div class="legend-dot" style="background:${info.color}"></div>${vid}`;
          container.appendChild(item);
        }
      });
    }

    function updateVehicles(data) {
      updateLegend(data);
      Object.entries(data).forEach(([vid, info]) => {
        const lat = info.lat, lon = info.lon;
        const color = info.color;
        const spd = (info.speed * 3.6).toFixed(1);

        if (!markers[vid]) {
          markers[vid] = L.marker([lat, lon], {icon: makeIcon(color)})
            .addTo(map).bindPopup('');
          trails[vid] = [];
        }
        markers[vid].setLatLng([lat, lon]);
        markers[vid].setPopupContent(`
          <b>${vid}</b><br>
          lat: ${lat.toFixed(6)}<br>
          lon: ${lon.toFixed(6)}<br>
          cap: ${info.heading.toFixed(1)}°<br>
          vitesse: ${spd} km/h
        `);

        trails[vid].push([lat, lon]);
        if (trails[vid].length > TRAIL_LEN) trails[vid].shift();
        if (trailLines[vid]) map.removeLayer(trailLines[vid]);
        trailLines[vid] = L.polyline(trails[vid], {
          color: color, weight: 2, opacity: 0.7
        }).addTo(map);
      });
    }

    let nUpdates = 0;
    setInterval(async () => {
      try {
        const res  = await fetch('/api/vehicles');
        const data = await res.json();
        updateVehicles(data);
        nUpdates++;
        document.getElementById('status').textContent =
          `✅ Live — ${Object.keys(data).length} véhicule(s) — #${nUpdates}`;
      } catch(e) {
        document.getElementById('status').textContent = '❌ Erreur connexion';
      }
    }, 200);
  </script>
</body>
</html>
"""


class MapServer(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
        elif self.path == '/api/vehicles':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            with state_lock:
                self.wfile.write(json.dumps(vehicles_state).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass


class CamListener(Node):
    def __init__(self):
        super().__init__('map_dashboard')
        self._color_idx = 0
        self._subs = {}

        # Découvrir les topics au démarrage puis toutes les 5s
        self._discover_and_subscribe()
        self.create_timer(5.0, self._discover_and_subscribe)
        self.get_logger().info(f'Dashboard → http://localhost:{WEB_PORT}')

    def _discover_and_subscribe(self):
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True, text=True, timeout=3)
            topics = [t.strip() for t in result.stdout.split('\n')
                      if '/its/cam/v' in t]
            for topic in topics:
                if topic not in self._subs:
                    color = COLORS[self._color_idx % len(COLORS)]
                    self._color_idx += 1
                    vid = topic.split('/')[-1]
                    self._subs[topic] = self.create_subscription(
                        CAM, topic,
                        lambda msg, v=vid, c=color: self._on_cam(msg, v, c),
                        10)
                    self.get_logger().info(f'Souscrit à {topic} ({color})')
        except Exception as e:
            self.get_logger().warn(f'Discovery error: {e}')

    def _on_cam(self, msg, vid, color):
        hfc = msg.cam.cam_parameters \
                  .high_frequency_container \
                  .basic_vehicle_container_high_frequency
        lat     = msg.cam.cam_parameters.basic_container \
                      .reference_position.latitude.value / 1e7
        lon     = msg.cam.cam_parameters.basic_container \
                      .reference_position.longitude.value / 1e7
        heading = hfc.heading.heading_value.value / 10.0
        speed   = hfc.speed.speed_value.value / 100.0

        with state_lock:
            vehicles_state[vid] = {
                'lat': lat, 'lon': lon,
                'heading': heading, 'speed': speed,
                'color': color,
            }


def main():
    rclpy.init()
    node = CamListener()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print(f'🌍 Dashboard: http://localhost:{WEB_PORT}')
    server = HTTPServer(('0.0.0.0', WEB_PORT), MapServer)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
