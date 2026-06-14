import math
import queue
import threading
import logging
import numpy as np
import carla
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField, NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header, String
from carla_multi_agent.pointpillar_client import PointPillarClient
from carla_multi_agent.fusion_detector import FusionDetector

logger = logging.getLogger(__name__)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class VehicleAgent(Node):
    def __init__(self, vehicle, vehicle_index, world, cfg):
        namespace = f"vehicle_{vehicle_index + 1}"
        super().__init__(f"vehicle_agent_{vehicle_index + 1}", namespace=namespace)
        self._vehicle_index = vehicle_index
        self.vehicle = vehicle
        self.cfg = cfg
        self.namespace = namespace
        self._sensors = []
        self._q_camera = queue.Queue(maxsize=2)
        self._q_lidar  = queue.Queue(maxsize=10)
        self._lidar_lock = threading.Lock()
        self._lidar_pub_queue = queue.Queue(maxsize=1)
        self._lidar_pub_thread = threading.Thread(target=self._lidar_publish_worker, daemon=True)
        self._pp_client = PointPillarClient() if vehicle_index == 0 else None
        self._semantic_det = None  # initialisé après attach_sensors
        self._detection_callback = None  # set externalement si besoin
        self._lidar_pub_thread.start()
        self._q_gnss   = queue.Queue(maxsize=5)
        self._q_imu    = queue.Queue(maxsize=5)
        self._pub_camera = self.create_publisher(Image,        "camera/image_raw", SENSOR_QOS)
        self._pub_lidar  = self.create_publisher(PointCloud2,  "lidar/points",     SENSOR_QOS)
        self._pub_gnss   = self.create_publisher(NavSatFix,    "gnss/fix",         SENSOR_QOS)
        self._pub_imu    = self.create_publisher(Imu,          "imu/data",         SENSOR_QOS)
        self._pub_odom   = self.create_publisher(Odometry,     "odometry",         10)
        self._pub_status = self.create_publisher(TwistStamped, "vehicle_status",   10)
        if vehicle_index == 0:
            self._pub_perceived = self.create_publisher(String, "/carla/hero/perceived_objects", 10)
        self._pub_transform = self.create_publisher(TransformStamped, "vehicle_transform", 10)
        c = cfg["sensors"]
        self.create_timer(1.0 / c["camera"]["publish_rate"], self._timer_camera)
        self.create_timer(1.0 / c["lidar"]["publish_rate"],  self._timer_lidar)
        self.create_timer(0.1,                               self._timer_gnss)
        self.create_timer(1.0 / c["imu"]["publish_rate"],    self._timer_imu)
        self.create_timer(0.05,                              self._timer_odom_status)
        self.create_timer(0.1,                               self._timer_transform)
        bp_lib = world.get_blueprint_library()
        self._attach_sensors(world, bp_lib, c)
        if self._vehicle_index == 0:
            self._semantic_det = FusionDetector(world, vehicle, cfg)
        logger.info("[%s] pret (%d capteurs)", namespace, len(self._sensors))

    def _attach_sensors(self, world, bp_lib, c):
        if c["camera"]["enabled"] and self._vehicle_index == 0:
            # Caméra RGB
            bp = bp_lib.find("sensor.camera.rgb")
            bp.set_attribute("image_size_x", str(c["camera"]["width"]))
            bp.set_attribute("image_size_y", str(c["camera"]["height"]))
            bp.set_attribute("fov",          str(c["camera"]["fov"]))
            s = world.spawn_actor(bp, carla.Transform(carla.Location(x=2.0, z=1.4)), attach_to=self.vehicle)
            s.listen(lambda d: self._camera_callback(d))
            self._sensors.append(s)

        if c["lidar"]["enabled"] and self._vehicle_index == 0:
            bp = bp_lib.find("sensor.lidar.ray_cast")
            bp.set_attribute("channels",           str(c["lidar"]["channels"]))
            bp.set_attribute("range",              str(c["lidar"]["range"]))
            bp.set_attribute("points_per_second",  str(c["lidar"]["points_per_second"]))
            bp.set_attribute("rotation_frequency", str(c["lidar"]["rotation_frequency"]))
            bp.set_attribute("upper_fov",          "2.0")
            bp.set_attribute("lower_fov",          "-26.0")
            s = world.spawn_actor(bp, carla.Transform(carla.Location(z=2.5)), attach_to=self.vehicle)
            s.listen(self._lidar_callback)
            self._sensors.append(s)
        if c["gnss"]["enabled"]:
            bp = bp_lib.find("sensor.other.gnss")
            s = world.spawn_actor(bp, carla.Transform(carla.Location(z=2.8)), attach_to=self.vehicle)
            s.listen(lambda d: self._q_put(self._q_gnss, d))
            self._sensors.append(s)
        if c["imu"]["enabled"]:
            bp = bp_lib.find("sensor.other.imu")
            s = world.spawn_actor(bp, carla.Transform(), attach_to=self.vehicle)
            s.listen(lambda d: self._q_put(self._q_imu, d))
            self._sensors.append(s)

    @staticmethod
    def _q_put(q, item):
        if q.full():
            try: q.get_nowait()
            except queue.Empty: pass
        try: q.put_nowait(item)
        except queue.Full: pass

    def _header(self, suffix=""):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = f"{self.namespace}{suffix}"
        return h

    def _timer_camera(self):
        try: image = self._q_camera.get_nowait()
        except queue.Empty:
            self.get_logger().debug("Camera queue empty (no frame yet)")
            return
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        rgb = arr[:, :, :3][:, :, ::-1]
        msg = Image()
        msg.header = self._header("/camera_link")
        msg.height = image.height
        msg.width  = image.width
        msg.encoding = "rgb8"
        msg.step = image.width * 3
        msg.data = rgb.tobytes()
        self._pub_camera.publish(msg)

    def _lidar_callback(self, data):
        """Dual path: ROS2 publish + direct PointPillars inference."""
        try:
            pts = np.frombuffer(data.raw_data, dtype=np.float32).reshape((-1, 4)).copy()
            # Path 1: ROS2 publish pour ExFMD-CP/Artery
            msg = PointCloud2()
            msg.header = self._header("/lidar_link")
            msg.height = 1
            msg.width  = pts.shape[0]
            msg.fields = [
                PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 16
            msg.row_step   = 16 * pts.shape[0]
            msg.data       = pts.tobytes()
            msg.is_dense   = True
            self._pub_lidar.publish(msg)
            # Path 2: inference directe sans overhead DDS
            if self._pp_client is not None:
                threading.Thread(target=self._run_detection, args=(pts,), daemon=True).start()
        except Exception as e:
            pass

    def _run_detection(self, pts):
        try:
            if self._semantic_det is not None:
                objects = self._semantic_det.detect()
            elif self._pp_client is not None:
                objects = self._pp_client.detect(pts)
            else:
                objects = []
            if objects:
                print(f"[DET] {len(objects)} objets: {[(o['class'], round(o['score'],2), round(o['distance'],1)) for o in objects]}", flush=True)
            # Publier sur /carla/hero/perceived_objects pour Artery
            if hasattr(self, '_pub_perceived'):
                import json as _json
                msg = String()
                msg.data = _json.dumps({'objects': objects, 'count': len(objects)})
                self._pub_perceived.publish(msg)
            if objects and self._detection_callback:
                self._detection_callback(objects)
        except Exception as e:
            print(f"[DET ERROR] {e}", flush=True)

    def _lidar_publish_worker(self):
        """No longer used."""
        while True:
            try:
                self._lidar_pub_queue.get()
                self._lidar_pub_queue.task_done()
            except Exception:
                pass


    def _timer_lidar(self):
        try: data = self._q_lidar.get_nowait()
        except queue.Empty:
            return
        pts = np.frombuffer(data.raw_data, dtype=np.float32).reshape((-1, 4)).copy()
        # pts[:, 2] += 1.5  # supprimé — offset géré par transform
        msg = PointCloud2()
        msg.header = self._header("/lidar_link")
        msg.height = 1
        msg.width  = pts.shape[0]
        msg.fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step   = 16 * pts.shape[0]
        msg.data       = pts.tobytes()
        msg.is_dense   = True
        self._pub_lidar.publish(msg)

    def _timer_gnss(self):
        try: data = self._q_gnss.get_nowait()
        except queue.Empty:
            self.get_logger().debug("Camera queue empty (no frame yet)")
            return
        msg = NavSatFix()
        msg.header = self._header("/gnss_link")
        msg.latitude  = data.latitude
        msg.longitude = data.longitude
        msg.altitude  = data.altitude
        msg.status.status  = 0
        msg.status.service = 1
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._pub_gnss.publish(msg)

    def _timer_imu(self):
        try: data = self._q_imu.get_nowait()
        except queue.Empty:
            self.get_logger().debug("Camera queue empty (no frame yet)")
            return
        msg = Imu()
        msg.header = self._header("/imu_link")
        msg.linear_acceleration.x = data.accelerometer.x
        msg.linear_acceleration.y = data.accelerometer.y
        msg.linear_acceleration.z = data.accelerometer.z
        msg.angular_velocity.x    = data.gyroscope.x
        msg.angular_velocity.y    = data.gyroscope.y
        msg.angular_velocity.z    = data.gyroscope.z
        yaw = math.radians(-data.compass)
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)
        msg.orientation_covariance[0] = -1.0
        self._pub_imu.publish(msg)

    def update_state(self, transform, velocity, angular_velocity, control):
        self._state = (transform, velocity, angular_velocity, control)
        # Publier la transform pour carla_perception_bridge
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = self.get_namespace().strip("/") + "/base_link"
        msg.transform.translation.x = float(transform.location.x)
        msg.transform.translation.y = float(transform.location.y)
        msg.transform.translation.z = float(transform.location.z)
        import math
        yaw = math.radians(transform.rotation.yaw)
        msg.transform.rotation.z = math.sin(yaw/2)
        msg.transform.rotation.w = math.cos(yaw/2)
        self._pub_transform.publish(msg)


    def _timer_transform(self):
        """Publie vehicle_transform à 10Hz depuis _state."""
        if not hasattr(self, "_state"):
            return
        t, v, av, ctrl = self._state
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = self.get_namespace().strip("/") + "/base_link"
        msg.transform.translation.x = float(t.location.x)
        msg.transform.translation.y = float(-t.location.y)
        msg.transform.translation.z = float(t.location.z)
        yaw = math.radians(-t.rotation.yaw)
        msg.transform.rotation.z = math.sin(yaw / 2.0)
        msg.transform.rotation.w = math.cos(yaw / 2.0)
        self._pub_transform.publish(msg)

    def _timer_odom_status(self):
        if not hasattr(self, "_state"): return
        t, v, av, ctrl = self._state
        speed_kmh = (v.x**2 + v.y**2 + v.z**2) ** 0.5 * 3.6
        odom = Odometry()
        odom.header = self._header("/odom")
        odom.child_frame_id = f"{self.namespace}/base_link"
        odom.pose.pose.position.x = t.location.x
        odom.pose.pose.position.y = -t.location.y
        odom.pose.pose.position.z = t.location.z
        yaw = math.radians(-t.rotation.yaw)
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x  = v.x
        odom.twist.twist.linear.y  = -v.y
        odom.twist.twist.angular.z = -math.radians(av.z)
        self._pub_odom.publish(odom)
        status = TwistStamped()
        status.header = self._header("/base_link")
        status.twist.linear.x  = speed_kmh
        status.twist.angular.z = ctrl.steer
        status.twist.linear.y  = ctrl.throttle
        status.twist.linear.z  = ctrl.brake
        self._pub_status.publish(status)

    def destroy_agent(self):
        for s in self._sensors:
            if s.is_alive:
                s.stop()
                s.destroy()
        self._sensors.clear()
        if self.vehicle.is_alive:
            self.vehicle.set_autopilot(False)
            self.vehicle.destroy()
        logger.info("[%s] detruit", self.namespace)

    def _camera_callback(self, image):
        if self._q_camera.full():
            try:
                self._q_camera.get_nowait()
            except queue.Empty:
                pass
        self._q_camera.put(image)
