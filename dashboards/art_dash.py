"""
map_dashboard.py
Souscrit aux topics /simulator/vehicle{n}/cam (ByteMultiArray 85 bytes)
décode le paquet GeoNet+BTP+CAM et affiche les véhicules sur OpenStreetMap.
"""
import threading
import json
import struct
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import ByteMultiArray

WEB_PORT = 8888

CAM_TOPICS = [
    ('/simulator/vehicle1/cam', 'v1', '#3498db'),
    ('/simulator/vehicle2/cam', 'v2', '#2ecc71'),
    ('/simulator/vehicle3/cam', 'v3', '#f39c12'),
]

vehicles_state = {}
state_lock = threading.Lock()


def decode_cam_bytes(data: bytes):
    """
    Décode un paquet GeoNet+BTP+CAM (85 bytes).
    Structure :
      GeoNet Basic Header   : 4 bytes
      GeoNet Common Header  : 8 bytes
      SHB Extended Header   : 28 bytes  → total GeoNet = 40 bytes
      BTP-B Header          : 4 bytes   → offset CAM = 44
      CAM ASN.1 UPER        : 41 bytes
    """
    if len(data) < 85:
        return None

    cam = data[44:]  # skip GeoNet (40) + BTP-B (4)

    try:
        # bytes 0-1 : protocolVersion + messageID
        protocol_version = cam[0]
        message_id = cam[1]
        if message_id != 2:  # 2 = CAM
            return None

        # bytes 2-5 : stationID (uint32 big-endian)
        station_id = int.from_bytes(cam[2:6], 'big')

        # bytes 6-7 : generationDeltaTime
        # bytes 8-11 : latitude (int32 big-endian, /1e7)
        lat = int.from_bytes(cam[8:12], 'big', signed=True) / 1e7

        # bytes 12-15 : longitude (int32 big-endian, /1e7)
        lon = int.from_bytes(cam[12:16], 'big', signed=True) / 1e7

        # Validation plausibilité
        if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
            return None

        return {
            'station_id': station_id,
            'lat': lat,
            'lon': lon,
        }
    except Exception:
        return None


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
    }
    .legend-item { display: flex; align-items: center; margin: 4px 0; }
    .legend-dot { width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; flex-shrink: 0; }
  </style>
</head>
<body>
  <div id="header">
    <h1>ExFMD-CP — Live Vehicle Tracking</h1>
    <span id="status">Connexion...</span>
  </div>
  <div id="map"></div>
  <div id="legend">
    <div style="font-weight:bold;margin-bottom:6px;">Véhicules</div>
    <div class="legend-item"><div class="legend-dot" style="background:#3498db"></div>Véhicule 1</div>
    <div class="legend-item"><div class="legend-dot" style="background:#2ecc71"></div>Véhicule 2</div>
    <div class="legend-item"><div class="legend-dot" style="background:#f39c12"></div>Véhicule 3</div>
  </div>
  <script>
    const map = L.map('map').setView([50.31841, 3.51045], 17);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© OpenStreetMap contributors', maxZoom: 19
    }).addTo(map);

    const markers = {}, trails = {}, trailLines = {};
    const TRAIL_LEN = 100;
    const COLORS = { v1:'#3498db', v2:'#2ecc71', v3:'#f39c12' };

    function makeIcon(color) {
      return L.divIcon({
        className: '',
        html: '<div style="width:16px;height:16px;border-radius:50%;background:'+color+';border:2px solid white;box-shadow:0 0 8px '+color+';"></div>',
        iconSize: [16,16], iconAnchor: [8,8]
      });
    }

    function updateVehicles(data) {
      Object.entries(data).forEach(([vid, info]) => {
        const lat = info.lat, lon = info.lon;
        const color = COLORS[vid] || '#fff';

        if (!markers[vid]) {
          markers[vid] = L.marker([lat, lon], {icon: makeIcon(color)})
            .addTo(map).bindPopup('');
          trails[vid] = [];
        }
        markers[vid].setLatLng([lat, lon]);
        markers[vid].setPopupContent(
          '<b>' + vid + '</b><br>' +
          'lat: ' + lat.toFixed(6) + '<br>' +
          'lon: ' + lon.toFixed(6) + '<br>' +
          'station_id: ' + info.station_id
        );

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
        const res = await fetch('/api/vehicles');
        const data = await res.json();
        updateVehicles(data);
        nUpdates++;
        document.getElementById('status').textContent =
          'Live — ' + Object.keys(data).length + ' véhicule(s) — #' + nUpdates;
      } catch(e) {
        document.getElementById('status').textContent = 'Erreur connexion';
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

        for topic, vid, color in CAM_TOPICS:
            self.create_subscription(
                ByteMultiArray,
                topic,
                lambda msg, v=vid, c=color: self._on_cam(msg, v, c),
                10)
            self.get_logger().info(f'Souscrit à {topic} ({color})')

        self.get_logger().info(f'Dashboard → http://localhost:{WEB_PORT}')

    def _on_cam(self, msg: ByteMultiArray, vid: str, color: str):
        try:
            raw = bytes([b[0] if isinstance(b, (bytes, bytearray)) else b for b in msg.data])
            decoded = decode_cam_bytes(raw)
            if decoded is None:
                return
            with state_lock:
                vehicles_state[vid] = {
                    'lat':        decoded['lat'],
                    'lon':        decoded['lon'],
                    'station_id': decoded['station_id'],
                    'color':      color,
                }
        except Exception as e:
            self.get_logger().warn(f'decode error [{vid}]: {e}')


def main():
    rclpy.init()
    node = CamListener()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print(f'Dashboard: http://localhost:{WEB_PORT}')
    server = HTTPServer(('0.0.0.0', WEB_PORT), MapServer)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
