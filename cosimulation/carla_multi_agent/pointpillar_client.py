import socket, json, struct
import numpy as np

class PointPillarClient:
    def __init__(self, host='127.0.0.1', port=9999):
        self.host = host
        self.port = port

    def detect(self, points_xyzI):
        if points_xyzI is None or len(points_xyzI) == 0:
            return []
        try:
            pts = points_xyzI[:, :4].astype(np.float32)
            data = pts.tobytes()
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.host, self.port))
                s.sendall(struct.pack('I', len(data)) + data)
                size = struct.unpack('I', s.recv(4))[0]
                result = b''
                while len(result) < size:
                    result += s.recv(size - len(result))
            res = json.loads(result.decode())
            return res if res is not None else []
        except Exception as e:
            print(f'[PointPillarClient] Error: {e}')
            return []
