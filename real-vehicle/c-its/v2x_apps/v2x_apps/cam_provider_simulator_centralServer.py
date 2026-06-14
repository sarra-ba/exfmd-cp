#!/usr/bin/env python3
"""
cam_provider_simulator_centralServer publishes test CAM payloads on vehicle topics,
collects them, and forwards snapshots to the central server endpoint.
"""

import asyncio
import json
import math
import re
import threading
import time
from typing import Any, Dict, Iterable, List, Optional, Tuple

import rclpy
from etsi_its_cam_msgs.msg import CAM
from rclpy.node import Node
import websockets

_DEFAULT_TTL_SECONDS = 7.0
_DEFAULT_DISCOVERY_PERIOD_SECONDS = 1.0
_DEFAULT_CLEANUP_PERIOD_SECONDS = 1.0
_DEFAULT_RECONNECT_MAX_BACKOFF_SECONDS = 30.0
_DEFAULT_TEST_PUBLISH_PERIOD_SECONDS = 1.0

# Visible test CAM payload template for /vehicle1/cam, /vehicle2/cam, /vehicle3/cam
TEST_CAM_PAYLOAD_TEMPLATE: Dict[str, Any] = {
    'header': {
        'protocol_version': 2,
        'message_id': 2,
        'station_id': {'value': 427949455},
    },
    'cam': {
        'generation_delta_time': {'value': 48837},
        'cam_parameters': {
            'basic_container': {
                'station_type': {'value': 5},
                'reference_position': {
                    'latitude': {'value': 503191663},
                    'longitude': {'value': 35104053},
                    'position_confidence_ellipse': {
                        'semi_major_confidence': {'value': 1580},
                        'semi_minor_confidence': {'value': 532},
                        'semi_major_orientation': {'value': 1777},
                    },
                    'altitude': {
                        'altitude_value': {'value': 12548},
                        'altitude_confidence': {'value': 9},
                    },
                },
            },
            'high_frequency_container': {
                'basic_vehicle_container_high_frequency': {
                    'heading': {'heading_value': {'value': 3513}, 'heading_confidence': {'value': 126}},
                    'speed': {'speed_value': {'value': 0}, 'speed_confidence': {'value': 52}},
                    'drive_direction': {'value': 2},
                    'vehicle_length': {
                        'vehicle_length_value': {'value': 19},
                        'vehicle_length_confidence_indication': {'value': 4},
                    },
                    'vehicle_width': {'value': 11},
                    'longitudinal_acceleration': {
                        'longitudinal_acceleration_value': {'value': 161},
                        'longitudinal_acceleration_confidence': {'value': 102},
                    },
                    'curvature': {'curvature_value': {'value': 1023}, 'curvature_confidence': {'value': 7}},
                    'curvature_calculation_mode': {'value': 2},
                    'yaw_rate': {'yaw_rate_value': {'value': 32767}, 'yaw_rate_confidence': {'value': 8}},
                },
            },
        },
    },
}

VEHICLE_TOPICS = ['/vehicle1/cam', '/vehicle2/cam', '/vehicle3/cam']
VEHICLE_TRAJECTORIES: Dict[str, List[Tuple[float, float]]] = {
    '1': [
        (50.31859264256525, 3.51129930955301),
        (50.31856100478464, 3.511232064828846),
        (50.31850224884954, 3.5112002120643524),
        (50.31847287085483, 3.511210829652839),
        (50.318450272384666, 3.5112356040249892),
        (50.31843445344933, 3.5112780743770315),
        (50.318423154206755, 3.511355936689654),
        (50.31843671329764, 3.5114408773936816),
        (50.31848191023957, 3.511483347745724),
        (50.31852484729447, 3.511483347745724),
        (50.31856100478464, 3.5114727301581468),
        (50.318594902406005, 3.5114196422176462),
        (50.31859264256525, 3.511355936689654),
        (50.318594902406005, 3.5113170055328737)
    ],
    '2': [
        (50.31874763775443, 3.5116008449068943),
        (50.31874143440541, 3.5115198844109727),
        (50.31870834986199, 3.511500453892154),
        (50.31868353643955, 3.5115198844109727),
        (50.318662858577824, 3.511558745449321),
        (50.31863390955587, 3.5116235138448246),
        (50.318588418199965, 3.5116623748832296),
        (50.31854913017594, 3.5116818054020484),
        (50.31849950315194, 3.5116688517223054),
        (50.3184540116674, 3.5116235138448246),
        (50.31842919811223, 3.5116073217459416),
        (50.318379570963, 3.511613798585813),
        (50.31834028276637, 3.5116332291046604),
        (50.31828651991833, 3.5116526596234507),
        (50.31825550286277, 3.511672090142241),
        (50.31823482481434, 3.511710951179822),
        (50.31828238431197, 3.5117400969584196),
        (50.31833201156263, 3.5117530506373384),
        (50.31838784215799, 3.511782196415936),
        (50.31844160489143, 3.5117983885148476),
        (50.31848709638783, 3.5118113421945623),
        (50.318532587840195, 3.5118210574535738),
        (50.31856980808766, 3.5118372495524284),
        (50.3186194350383, 3.5118437263923),
        (50.31866906193707, 3.5118437263923),
        (50.318722824352506, 3.511814580613702),
        (50.318735231055086, 3.5117692427370173),
        (50.318735231055086, 3.511717428019665),
        (50.31874143440541, 3.511649421203515)
    ],
    '3': [
        (50.318612741476926, 3.5102888956824643),
        (50.318608972337984, 3.5102210117615016),
        (50.31859954948979, 3.510170836690037),
        (50.318593895779685, 3.5101206616185436),
        (50.31858635749899, 3.5100439232735425),
        (50.318580703787774, 3.5099583305044746),
        (50.318571280933526, 3.509896349533335),
        (50.31856374264922, 3.509816659712669),
        (50.31855808893491, 3.5097605816913244),
        (50.31854489693259, 3.509660231548395),
        (50.31854301236089, 3.5095746387792985),
        (50.318535474071666, 3.5094949489586327),
        (50.31852228206313, 3.5094477253628042),
        (50.31851474377103, 3.50934442374421),
        (50.31850532090414, 3.509282442773099),
        (50.31849589803497, 3.509202752952433),
        (50.31849024431298, 3.50915257788094),
        (50.318469513992625, 3.509087645435585),
        (50.318441245359026, 3.509087645435585),
        (50.31840920755431, 3.509093548385408),
        (50.31839036177641, 3.5091466749311166),
        (50.318363977675034, 3.5092057044280693),
        (50.31836209309577, 3.5093031030967836),
        (50.31835643936867, 3.5093798416144466),
        (50.318347016470454, 3.50941525931205),
        (50.318333824409535, 3.5094742888090025),
        (50.318322516925974, 3.509536269780142),
        (50.31831686318287, 3.5096071051753484),
        (50.318301786531975, 3.509669086146488),
        (50.31829424820495, 3.5097281156420195),
        (50.318292363622874, 3.5097900966131874),
        (50.31828294071195, 3.509840271684652),
        (50.31826409488397, 3.5098874952819017),
        (50.31826032571786, 3.509940621829003),
        (50.31824901821673, 3.5099966998503476),
        (50.31824901821673, 3.510046874921784),
        (50.318247133632894, 3.5100881955692103),
        (50.31825844113442, 3.5101531280145934),
        (50.31827728696419, 3.510218060459948),
        (50.31828105612942, 3.5102770899568725),
        (50.31829424820495, 3.5103243135541504),
        (50.318303671113654, 3.5103921974737204),
        (50.31831686318287, 3.5104217122228647),
        (50.31833570898951, 3.5104364695968115),
        (50.31837528515959, 3.510424663697137),
        (50.31840732298764, 3.510395148949385),
        (50.31845255282519, 3.510380391575495),
        (50.31851285920851, 3.5103449738778636),
        (50.31856939637322, 3.5103272650283657),
        (50.318595780360596, 3.5103036532304657)
    ]
}


def _safe_nested_attr(root: Any, path: Iterable[str]) -> Any:
    node = root
    for attr in path:
        if node is None or not hasattr(node, attr):
            return None
        node = getattr(node, attr)
    return node


def _extract_first(root: Any, paths: Iterable[Tuple[str, ...]]) -> Any:
    for path in paths:
        value = _safe_nested_attr(root, path)
        if value is not None:
            return value
    return None


def _to_float_or_none(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_str_or_none(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text if text else None


def _scaled_coord_to_decimal(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    if abs(value) > 180.0:
        return value / 1e7
    return value


def _etsi_heading_to_degrees(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    return value * 0.1


def _etsi_speed_to_mps(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    return value * 0.01


class CamStore:
    def __init__(self) -> None:
        self._records: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()

    @staticmethod
    def _now() -> float:
        return time.time()

    def upsert(self, key: str, record: Dict[str, Any]) -> None:
        with self._lock:
            self._records[key] = record

    def prune_stale(self, ttl_seconds: float) -> bool:
        now = self._now()
        changed = False
        with self._lock:
            stale_keys = [
                key for key, item in self._records.items()
                if (now - item.get('updated_at', 0.0)) > ttl_seconds
            ]
            for key in stale_keys:
                del self._records[key]
                changed = True
        return changed

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            cams = [dict(item) for item in self._records.values()]
        return {'timestamp': self._now(), 'cams': cams}


class SnapshotForwarder:
    def __init__(self, endpoint_url: str, max_backoff_seconds: float, logger) -> None:
        self._endpoint_url = endpoint_url
        self._max_backoff_seconds = max_backoff_seconds
        self._logger = logger
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._condition = threading.Condition()
        self._pending_payload: Optional[str] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def publish_snapshot(self, snapshot: Dict[str, Any]) -> None:
        payload = json.dumps(snapshot, separators=(',', ':'))
        with self._condition:
            self._pending_payload = payload
            self._condition.notify_all()

    def _wait_for_payload(self, timeout: float = 1.0) -> Optional[str]:
        with self._condition:
            self._condition.wait_for(
                lambda: self._stop_event.is_set() or self._pending_payload is not None,
                timeout=timeout,
            )
            if self._pending_payload is None:
                return None
            payload = self._pending_payload
            self._pending_payload = None
            return payload

    async def _run_sender(self) -> None:
        backoff_seconds = 1.0
        while not self._stop_event.is_set():
            try:
                async with websockets.connect(self._endpoint_url, ping_interval=20, ping_timeout=20) as ws:
                    self._logger.info(f'Connected to central server: {self._endpoint_url}')
                    backoff_seconds = 1.0
                    while not self._stop_event.is_set():
                        payload = await asyncio.to_thread(self._wait_for_payload, 1.0)
                        if payload is None:
                            continue
                        await ws.send(payload)
            except Exception as error:
                self._logger.warning(
                    f'Central server connection failed ({error}); retrying in {backoff_seconds:.1f}s'
                )
                await asyncio.sleep(backoff_seconds)
                backoff_seconds = min(backoff_seconds * 2.0, self._max_backoff_seconds)

    def _thread_main(self) -> None:
        asyncio.run(self._run_sender())


class CamProviderSimulatorCentralServer(Node):
    def __init__(self) -> None:
        super().__init__('cam_provider_simulator_central_server')
        self._store = CamStore()
        self._cam_subscriptions: Dict[str, Any] = {}
        self._test_publishers: Dict[str, Any] = {}
        self._trajectory_indexes: Dict[str, int] = {}
        self._compiled_patterns: List[re.Pattern[str]] = []
        self._vehicle_topic_re = re.compile(r'^/vehicle(?P<vehicle_id>\d+)/cam$')

        self.declare_parameter('input_topics', ['/its/cam_received'])
        self.declare_parameter('dynamic_topic_patterns', [r'^/vehicle\d+/cam$'])
        self.declare_parameter('data_ttl_seconds', _DEFAULT_TTL_SECONDS)
        self.declare_parameter('discovery_period_seconds', _DEFAULT_DISCOVERY_PERIOD_SECONDS)
        self.declare_parameter('cleanup_period_seconds', _DEFAULT_CLEANUP_PERIOD_SECONDS)
        self.declare_parameter('central_ws_ingest_url', 'ws://127.0.0.1:8010/ws/ingest')
        self.declare_parameter('reconnect_max_backoff_seconds', _DEFAULT_RECONNECT_MAX_BACKOFF_SECONDS)
        self.declare_parameter('test_publish_period_seconds', _DEFAULT_TEST_PUBLISH_PERIOD_SECONDS)

        self._ttl_seconds = float(self.get_parameter('data_ttl_seconds').value)
        self._forwarder = SnapshotForwarder(
            endpoint_url=str(self.get_parameter('central_ws_ingest_url').value),
            max_backoff_seconds=float(self.get_parameter('reconnect_max_backoff_seconds').value),
            logger=self.get_logger(),
        )

        self._load_patterns()
        self._subscribe_configured_topics()
        self._create_test_publishers()
        self.create_timer(
            float(self.get_parameter('discovery_period_seconds').value),
            self._discover_topics,
        )
        self.create_timer(
            float(self.get_parameter('cleanup_period_seconds').value),
            self._on_cleanup_timer,
        )
        self.create_timer(
            float(self.get_parameter('test_publish_period_seconds').value),
            self._publish_test_cam_topics,
        )

        self._forwarder.start()
        self.get_logger().info(f'Node "{self.get_name()}" started')

    def _load_patterns(self) -> None:
        patterns = self.get_parameter('dynamic_topic_patterns').value or []
        self._compiled_patterns = []
        for item in patterns:
            try:
                self._compiled_patterns.append(re.compile(str(item)))
            except re.error:
                self.get_logger().warning(f'Ignoring invalid regex pattern: {item}')

    def _subscribe_configured_topics(self) -> None:
        topics = self.get_parameter('input_topics').value or []
        for topic in topics:
            self._subscribe_topic(str(topic))

    def _create_test_publishers(self) -> None:
        for topic in VEHICLE_TOPICS:
            self._test_publishers[topic] = self.create_publisher(CAM, topic, 10)
            self._trajectory_indexes[topic] = 0
            self.get_logger().info(f'Test CAM publisher ready: {topic}')

    def _should_track_topic(self, topic_name: str, topic_types: List[str]) -> bool:
        if 'etsi_its_cam_msgs/msg/CAM' not in topic_types:
            return False
        for pattern in self._compiled_patterns:
            if pattern.match(topic_name):
                return True
        return False

    def _discover_topics(self) -> None:
        for topic_name, topic_types in self.get_topic_names_and_types():
            if self._should_track_topic(topic_name, topic_types):
                self._subscribe_topic(topic_name)

    def _subscribe_topic(self, topic_name: str) -> None:
        # Vérification avec le nouveau nom
        if topic_name in self._cam_subscriptions:
            return
        callback = lambda msg, topic=topic_name: self._on_cam(msg, topic)
        
        # Stockage dans votre dictionnaire renommé
        self._cam_subscriptions[topic_name] = self.create_subscription(CAM, topic_name, callback, 10)
        self.get_logger().info(f'Subscribed to CAM topic: {topic_name}')

    def _extract_vehicle_id(self, topic_name: str, station_id: Optional[str]) -> Optional[str]:
        match = self._vehicle_topic_re.match(topic_name)
        if match:
            return match.group('vehicle_id')
        return station_id

    @staticmethod
    def _clamp(value: int, minimum: int, maximum: int) -> int:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _to_cam_latitude(latitude_deg: float) -> int:
        return CamProviderSimulatorCentralServer._clamp(int(round(latitude_deg * 1e7)), -900000000, 900000001)

    @staticmethod
    def _to_cam_longitude(longitude_deg: float) -> int:
        return CamProviderSimulatorCentralServer._clamp(int(round(longitude_deg * 1e7)), -1800000000, 1800000001)

    @staticmethod
    def _bearing_deg(start: Tuple[float, float], end: Tuple[float, float]) -> float:
        lat1 = math.radians(start[0])
        lat2 = math.radians(end[0])
        dlon = math.radians(end[1] - start[1])
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    @staticmethod
    def _distance_m(start: Tuple[float, float], end: Tuple[float, float]) -> float:
        earth_radius_m = 6371000.0
        lat1 = math.radians(start[0])
        lat2 = math.radians(end[0])
        dlat = lat2 - lat1
        dlon = math.radians(end[1] - start[1])
        a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return earth_radius_m * c

    def _build_test_cam_message(
        self,
        vehicle_id: str,
        point: Tuple[float, float],
        next_point: Tuple[float, float],
        period_seconds: float,
    ) -> CAM:
        template = TEST_CAM_PAYLOAD_TEMPLATE
        heading_deg = self._bearing_deg(point, next_point)
        heading_value = self._clamp(int(round(heading_deg * 10.0)), 0, 3600)
        speed_mps = self._distance_m(point, next_point) / max(period_seconds, 0.1)
        speed_value = self._clamp(int(round(speed_mps * 100.0)), 0, 16382)

        msg = CAM()
        msg.header.protocol_version = template['header']['protocol_version']
        msg.header.message_id = template['header']['message_id']
        msg.header.station_id.value = int(template['header']['station_id']['value']) + int(vehicle_id)
        msg.cam.generation_delta_time.value = template['cam']['generation_delta_time']['value']

        basic = template['cam']['cam_parameters']['basic_container']
        ref = basic['reference_position']
        msg.cam.cam_parameters.basic_container.station_type.value = basic['station_type']['value']
        msg.cam.cam_parameters.basic_container.reference_position.latitude.value = self._to_cam_latitude(point[0])
        msg.cam.cam_parameters.basic_container.reference_position.longitude.value = self._to_cam_longitude(point[1])
        msg.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse.semi_major_confidence.value = ref['position_confidence_ellipse']['semi_major_confidence']['value']
        msg.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse.semi_minor_confidence.value = ref['position_confidence_ellipse']['semi_minor_confidence']['value']
        msg.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse.semi_major_orientation.value = ref['position_confidence_ellipse']['semi_major_orientation']['value']
        msg.cam.cam_parameters.basic_container.reference_position.altitude.altitude_value.value = ref['altitude']['altitude_value']['value']
        msg.cam.cam_parameters.basic_container.reference_position.altitude.altitude_confidence.value = ref['altitude']['altitude_confidence']['value']

        hf_template = template['cam']['cam_parameters']['high_frequency_container']['basic_vehicle_container_high_frequency']
        hf = msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency
        hf.heading.heading_value.value = heading_value
        hf.heading.heading_confidence.value = hf_template['heading']['heading_confidence']['value']
        hf.speed.speed_value.value = speed_value
        hf.speed.speed_confidence.value = hf_template['speed']['speed_confidence']['value']
        hf.drive_direction.value = hf_template['drive_direction']['value']
        hf.vehicle_length.vehicle_length_value.value = hf_template['vehicle_length']['vehicle_length_value']['value']
        hf.vehicle_length.vehicle_length_confidence_indication.value = hf_template['vehicle_length']['vehicle_length_confidence_indication']['value']
        hf.vehicle_width.value = hf_template['vehicle_width']['value']
        hf.longitudinal_acceleration.longitudinal_acceleration_value.value = hf_template['longitudinal_acceleration']['longitudinal_acceleration_value']['value']
        hf.longitudinal_acceleration.longitudinal_acceleration_confidence.value = hf_template['longitudinal_acceleration']['longitudinal_acceleration_confidence']['value']
        hf.curvature.curvature_value.value = hf_template['curvature']['curvature_value']['value']
        hf.curvature.curvature_confidence.value = hf_template['curvature']['curvature_confidence']['value']
        hf.curvature_calculation_mode.value = hf_template['curvature_calculation_mode']['value']
        hf.yaw_rate.yaw_rate_value.value = hf_template['yaw_rate']['yaw_rate_value']['value']
        hf.yaw_rate.yaw_rate_confidence.value = hf_template['yaw_rate']['yaw_rate_confidence']['value']
        return msg

    def _publish_test_cam_topics(self) -> None:
        period_seconds = float(self.get_parameter('test_publish_period_seconds').value)
        for topic, publisher in self._test_publishers.items():
            vehicle_id = topic.replace('/vehicle', '').replace('/cam', '')
            trajectory = VEHICLE_TRAJECTORIES.get(vehicle_id, [])
            if len(trajectory) < 2:
                continue
            index = self._trajectory_indexes[topic]
            next_index = (index + 1) % len(trajectory)
            cam_msg = self._build_test_cam_message(
                vehicle_id=vehicle_id,
                point=trajectory[index],
                next_point=trajectory[next_index],
                period_seconds=period_seconds,
            )
            publisher.publish(cam_msg)
            self._trajectory_indexes[topic] = next_index

    def _extract_cam_record(self, msg: CAM, source_topic: str) -> Optional[Tuple[str, Dict[str, Any]]]:
        station_id = _to_str_or_none(_extract_first(msg, (
            ('header', 'station_id', 'value'),
            ('header', 'station_id'),
        )))
        if station_id is None:
            return None

        lat_raw = _to_float_or_none(_extract_first(msg, (
            ('cam', 'cam_parameters', 'basic_container', 'reference_position', 'latitude', 'value'),
            ('payload', 'cam', 'cam_parameters', 'basic_container', 'reference_position', 'latitude', 'value'),
            ('payload', 'basic_container', 'reference_position', 'latitude', 'value'),
        )))
        lon_raw = _to_float_or_none(_extract_first(msg, (
            ('cam', 'cam_parameters', 'basic_container', 'reference_position', 'longitude', 'value'),
            ('payload', 'cam', 'cam_parameters', 'basic_container', 'reference_position', 'longitude', 'value'),
            ('payload', 'basic_container', 'reference_position', 'longitude', 'value'),
        )))
        heading_raw = _to_float_or_none(_extract_first(msg, (
            ('cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'heading', 'heading_value', 'value'),
            ('cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'heading', 'value', 'value'),
            ('cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'heading', 'value'),
            ('payload', 'cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'heading', 'heading_value', 'value'),
            ('payload', 'cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'heading', 'value', 'value'),
            ('payload', 'cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'heading', 'value'),
        )))
        speed_raw = _to_float_or_none(_extract_first(msg, (
            ('cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'speed', 'speed_value', 'value'),
            ('cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'speed', 'value', 'value'),
            ('cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'speed', 'value'),
            ('payload', 'cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'speed', 'speed_value', 'value'),
            ('payload', 'cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'speed', 'value', 'value'),
            ('payload', 'cam', 'cam_parameters', 'high_frequency_container', 'basic_vehicle_container_high_frequency', 'speed', 'value'),
        )))
        station_type = _to_float_or_none(_extract_first(msg, (
            ('cam', 'cam_parameters', 'basic_container', 'station_type', 'value'),
            ('payload', 'cam', 'cam_parameters', 'basic_container', 'station_type', 'value'),
            ('payload', 'basic_container', 'station_type', 'value'),
        )))
        rssi = _to_float_or_none(_extract_first(msg, (
            ('header', 'rssi'),
            ('header', 'rx_power'),
        )))
        generation_delta_ms = _to_float_or_none(_extract_first(msg, (
            ('generation_delta_time', 'value'),
            ('generation_delta_time',),
        )))
        now = time.time()
        vehicle_id = self._extract_vehicle_id(source_topic, station_id)
        key = station_id
        record = {
            'vehicle_id': vehicle_id,
            'station_id': station_id,
            'station_type': int(station_type) if station_type is not None else None,
            'latitude': _scaled_coord_to_decimal(lat_raw),
            'longitude': _scaled_coord_to_decimal(lon_raw),
            'heading': _etsi_heading_to_degrees(heading_raw),
            'speed': _etsi_speed_to_mps(speed_raw),
            'rssi': rssi,
            'generation_delta_time_ms': generation_delta_ms,
            'source_topic': source_topic,
            'updated_at': now,
            'timestamp': now,
        }
        return key, record

    def _publish_state(self) -> None:
        self._forwarder.publish_snapshot(self._store.snapshot())

    def _on_cam(self, msg: CAM, source_topic: str) -> None:
        extracted = self._extract_cam_record(msg, source_topic)
        if extracted is None:
            return
        key, record = extracted
        self._store.upsert(key, record)
        self._publish_state()

    def _on_cleanup_timer(self) -> None:
        if self._store.prune_stale(self._ttl_seconds):
            self._publish_state()

    def destroy_node(self) -> bool:
        self._forwarder.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CamProviderSimulatorCentralServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
