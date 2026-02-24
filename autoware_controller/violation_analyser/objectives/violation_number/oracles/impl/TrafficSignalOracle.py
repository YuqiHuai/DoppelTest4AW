from typing import Dict, List, Optional, Set

from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroupArray
from nav_msgs.msg import Odometry
from shapely.geometry import Polygon

from objectives.violation_number.oracles.OracleInterface import OracleInterface
from objectives.violation_number.oracles.Violation import Violation
from tools.autoware_tools.calculate_velocity import calculate_velocity
from tools.hdmap.VectorMapParser import VectorMapParser
from tools.utils import generate_adc_polygon, quaternion_2_heading


class TrafficSignalOracle(OracleInterface):
    """
    Red-light violation oracle.

    A violation is recorded when:
      - the ego vehicle intersects the stop line of a traffic signal,
      - that signal is currently RED,
      - and ego speed is positive.

    Event-level counting:
      - multiple signal IDs can share one physical stop line on map,
      - this oracle deduplicates such IDs into one violation event.
    """

    STOPPED_SPEED_MPS = 0.01

    last_localization: Optional[Odometry]
    last_traffic_signal_detection: Optional[TrafficLightGroupArray]
    red_signal_ids: Set[str]
    traffic_signal_stop_line_string_dict: Dict[str, object]
    signal_id_to_event_key: Dict[str, str]
    event_key_to_signal_ids: Dict[str, Set[str]]
    violated_event_keys: Set[str]
    violation_features: Dict[str, dict]

    def __init__(self) -> None:
        self.last_localization = None
        self.last_traffic_signal_detection = None
        self.red_signal_ids = set()
        self.traffic_signal_stop_line_string_dict = dict()
        self.signal_id_to_event_key = {}
        self.event_key_to_signal_ids = {}
        self.violated_event_keys = set()
        self.violation_features = {}
        self.parse_traffic_signal_stop_line_string_on_map()

    def get_interested_topics(self):
        return [
            "/localization/kinematic_state",
            "/perception/traffic_light_recognition/traffic_signals",
        ]

    def on_new_message(self, topic: str, message, t):
        if topic == "/localization/kinematic_state":
            self.last_localization = message
            self._check_violation()
            return
        if topic == "/perception/traffic_light_recognition/traffic_signals":
            self.last_traffic_signal_detection = message
            self.red_signal_ids = self._extract_red_signal_ids(message)
            self._check_violation()

    def _extract_red_signal_ids(self, message: TrafficLightGroupArray) -> Set[str]:
        result: Set[str] = set()
        groups = getattr(message, "traffic_light_groups", None) or []
        for group in groups:
            group_id = getattr(group, "traffic_light_group_id", None)
            if group_id is None:
                continue
            if self._is_group_red(group):
                result.add(str(group_id))
        return result

    @staticmethod
    def _is_group_red(group) -> bool:
        elements = getattr(group, "elements", None) or []
        for element in elements:
            color = getattr(element, "color", None)
            if color == TrafficLightElement.RED:
                return True
        return False

    @staticmethod
    def _normalize_linestring_coords(coords) -> Optional[str]:
        try:
            rounded = [(round(float(x), 3), round(float(y), 3)) for x, y in coords]
        except Exception:
            return None
        if len(rounded) < 2:
            return None
        reversed_coords = list(reversed(rounded))
        normalized = reversed_coords if tuple(reversed_coords) < tuple(rounded) else rounded
        return ";".join(f"{x:.3f},{y:.3f}" for x, y in normalized)

    @classmethod
    def _stop_line_event_key(cls, geom) -> Optional[str]:
        if geom is None:
            return None
        geom_type = getattr(geom, "geom_type", "")
        if geom_type == "LineString":
            key = cls._normalize_linestring_coords(list(geom.coords))
            return f"LS:{key}" if key else None
        if geom_type == "MultiLineString":
            parts: List[str] = []
            for line in getattr(geom, "geoms", []):
                key = cls._normalize_linestring_coords(list(line.coords))
                if key:
                    parts.append(key)
            if not parts:
                return None
            parts.sort()
            return "MLS:" + "|".join(parts)
        try:
            min_x, min_y, max_x, max_y = geom.bounds
            return (
                f"GEOM:{round(float(min_x), 3)}:{round(float(min_y), 3)}:"
                f"{round(float(max_x), 3)}:{round(float(max_y), 3)}"
            )
        except Exception:
            return None

    def _check_violation(self) -> None:
        if self.last_localization is None:
            return
        if not self.red_signal_ids:
            return
        if not self.traffic_signal_stop_line_string_dict:
            return

        intersecting_ids = self._intersecting_signal_ids()
        if not intersecting_ids:
            return

        speed = calculate_velocity(self.last_localization.twist.twist.linear)
        if speed <= self.STOPPED_SPEED_MPS:
            return

        intersecting_event_keys: Set[str] = set()
        for signal_id in intersecting_ids:
            event_key = self.signal_id_to_event_key.get(signal_id, f"signal:{signal_id}")
            intersecting_event_keys.add(event_key)

        for event_key in intersecting_event_keys:
            if event_key in self.violated_event_keys:
                continue
            signal_ids = sorted(self.event_key_to_signal_ids.get(event_key, set()))
            if not signal_ids:
                signal_ids = sorted(
                    [
                        sid
                        for sid in intersecting_ids
                        if self.signal_id_to_event_key.get(sid, f"signal:{sid}") == event_key
                    ]
                )
            representative_signal_id = signal_ids[0] if signal_ids else event_key
            features = self.get_basic_info_from_localization(self.last_localization)
            features["traffic_signal_id"] = representative_signal_id
            if len(signal_ids) > 1:
                features["traffic_signal_ids"] = signal_ids
            self.violation_features[event_key] = features
            self.violated_event_keys.add(event_key)

    def _intersecting_signal_ids(self) -> List[str]:
        adc_pose = self.last_localization.pose.pose
        adc_heading = quaternion_2_heading(adc_pose.orientation)
        adc_polygon_pts = generate_adc_polygon(adc_pose.position, adc_heading)
        adc_polygon = Polygon([[p.x, p.y] for p in adc_polygon_pts])

        result: List[str] = []
        for signal_id, stop_line_geom in self.traffic_signal_stop_line_string_dict.items():
            if signal_id not in self.red_signal_ids:
                continue
            try:
                # Buffer line-like stop-lines to avoid zero-area edge misses.
                geom = stop_line_geom
                geom_type = getattr(stop_line_geom, "geom_type", "")
                if geom_type in ("LineString", "MultiLineString"):
                    geom = stop_line_geom.buffer(0.2)
                if not geom.intersection(adc_polygon).is_empty:
                    result.append(signal_id)
            except Exception:
                continue
        return result

    def parse_traffic_signal_stop_line_string_on_map(self) -> None:
        self.traffic_signal_stop_line_string_dict = dict()
        self.signal_id_to_event_key = {}
        self.event_key_to_signal_ids = {}
        try:
            map_parser = VectorMapParser.instance()
            if not hasattr(map_parser, "lanelet_map") or map_parser.lanelet_map is None:
                return
        except Exception:
            return

        try:
            for ts_id in map_parser.get_signals():
                stop_line = map_parser.get_stop_line_for_signal(ts_id)
                if stop_line is not None:
                    signal_id = str(ts_id)
                    self.traffic_signal_stop_line_string_dict[signal_id] = stop_line
                    event_key = self._stop_line_event_key(stop_line) or f"signal:{signal_id}"
                    self.signal_id_to_event_key[signal_id] = event_key
                    if event_key not in self.event_key_to_signal_ids:
                        self.event_key_to_signal_ids[event_key] = set()
                    self.event_key_to_signal_ids[event_key].add(signal_id)
        except Exception:
            return

    def get_result(self) -> List[Violation]:
        result: List[Violation] = []
        for event_key in sorted(self.violated_event_keys):
            features = self.violation_features.get(event_key)
            signal_ids = sorted(self.event_key_to_signal_ids.get(event_key, set()))
            representative_signal_id = signal_ids[0] if signal_ids else event_key
            if features is None:
                if self.last_localization is not None:
                    features = self.get_basic_info_from_localization(self.last_localization)
                else:
                    features = self.get_dummy_basic_info()
                features["traffic_signal_id"] = representative_signal_id
                if len(signal_ids) > 1:
                    features["traffic_signal_ids"] = signal_ids
            result.append(
                Violation("TrafficSignalOracle", features, representative_signal_id)
            )
        return result
