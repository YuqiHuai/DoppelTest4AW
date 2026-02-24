from typing import Dict, List, Optional, Set, Tuple

from lanelet2.core import BasicPoint2d
from lanelet2.geometry import inside
from nav_msgs.msg import Odometry
from shapely.geometry import Polygon

from objectives.violation_number.oracles.OracleInterface import OracleInterface
from objectives.violation_number.oracles.Violation import Violation
from tools.autoware_tools.calculate_velocity import calculate_velocity
from tools.hdmap.VectorMapParser import VectorMapParser
from tools.utils import generate_adc_polygon, quaternion_2_heading


class StopSignOracle(OracleInterface):
    """
    Stop-sign oracle.

    A violation is recorded when ego enters a stop-sign stop line without
    having reached complete stop in the recent look-back window.
    """

    LOOKBACK_SECONDS = 5.0
    STOPPED_SPEED_MPS = 0.01
    MAX_HISTORY_SECONDS = 12.0

    last_localization: Optional[Odometry]
    history: List[Tuple[float, Odometry]]
    stop_sign_stop_line_dict: Dict[str, object]
    stop_sign_id_to_lane_ids: Dict[str, Set[int]]
    stop_sign_id_to_event_key: Dict[str, str]
    event_key_to_stop_sign_ids: Dict[str, Set[str]]
    violated_event_keys: Set[str]
    violation_features: Dict[str, dict]
    active_intersections: Set[str]
    checked_once: Set[str]

    def __init__(self) -> None:
        self.last_localization = None
        self.history = []
        self.stop_sign_stop_line_dict = {}
        self.stop_sign_id_to_lane_ids = {}
        self.stop_sign_id_to_event_key = {}
        self.event_key_to_stop_sign_ids = {}
        self.violated_event_keys = set()
        self.violation_features = {}
        self.active_intersections = set()
        self.checked_once = set()
        self.parse_stop_sign_stop_line_on_map()

    def get_interested_topics(self) -> List[str]:
        return ["/localization/kinematic_state"]

    def on_new_message(self, topic: str, message, t):
        if topic != "/localization/kinematic_state":
            return
        self.last_localization = message
        now_s = self._timestamp_from_message_or_bag(message, t)
        self.history.append((now_s, message))
        self._prune_history(now_s)

        intersecting_ids = self._intersecting_stop_sign_ids(message)
        intersecting_event_keys: Set[str] = set()
        for stop_sign_id in intersecting_ids:
            event_key = self.stop_sign_id_to_event_key.get(
                stop_sign_id, f"stop_sign:{stop_sign_id}"
            )
            intersecting_event_keys.add(event_key)

        newly_entered = intersecting_event_keys - self.active_intersections
        self.active_intersections = intersecting_event_keys

        for event_key in newly_entered:
            if event_key in self.checked_once:
                continue
            self.checked_once.add(event_key)
            if self._had_complete_stop_before(now_s):
                continue
            self.violated_event_keys.add(event_key)

            stop_sign_ids = sorted(self.event_key_to_stop_sign_ids.get(event_key, set()))
            representative_stop_sign_id = stop_sign_ids[0] if stop_sign_ids else event_key
            features = self.get_basic_info_from_localization(message)
            features["stop_sign_id"] = representative_stop_sign_id
            if len(stop_sign_ids) > 1:
                features["stop_sign_ids"] = stop_sign_ids
            self.violation_features[event_key] = features

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

    def _timestamp_from_message_or_bag(self, message: Odometry, t) -> float:
        try:
            stamp = message.header.stamp
            ts = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            if ts > 0:
                return ts
        except Exception:
            pass
        try:
            ts = float(t)
            if ts > 1e12:
                return ts * 1e-9
            return ts
        except Exception:
            return 0.0

    def _prune_history(self, now_s: float) -> None:
        if now_s <= 0:
            if len(self.history) > 500:
                self.history = self.history[-500:]
            return
        threshold = now_s - self.MAX_HISTORY_SECONDS
        self.history = [(ts, msg) for ts, msg in self.history if ts >= threshold]

    def _had_complete_stop_before(self, crossing_ts: float) -> bool:
        if not self.history:
            return False
        for ts, msg in reversed(self.history):
            if ts >= crossing_ts:
                continue
            if crossing_ts > 0 and (crossing_ts - ts) > self.LOOKBACK_SECONDS:
                break
            speed = calculate_velocity(msg.twist.twist.linear)
            if speed <= self.STOPPED_SPEED_MPS:
                return True
        return False

    def _intersecting_stop_sign_ids(self, localization: Odometry) -> Set[str]:
        if not self.stop_sign_stop_line_dict:
            return set()
        current_lanelet_ids = self._current_lanelet_ids(localization)
        route_lanelet_ids: Set[int] = set()
        if hasattr(self, "oh") and self.oh:
            route_lanelet_ids = self.oh.get_route_lanelet_ids()
        adc_pose = localization.pose.pose
        adc_heading = quaternion_2_heading(adc_pose.orientation)
        adc_polygon_pts = generate_adc_polygon(adc_pose.position, adc_heading)
        adc_polygon = Polygon([[p.x, p.y] for p in adc_polygon_pts])

        result: Set[str] = set()
        for stop_sign_id, stop_line_geom in self.stop_sign_stop_line_dict.items():
            controlled_lane_ids = set(self.stop_sign_id_to_lane_ids.get(stop_sign_id, set()))
            if route_lanelet_ids and controlled_lane_ids:
                controlled_lane_ids = controlled_lane_ids.intersection(route_lanelet_ids)
                if not controlled_lane_ids:
                    continue
            if current_lanelet_ids and controlled_lane_ids:
                if current_lanelet_ids.isdisjoint(controlled_lane_ids):
                    continue
            try:
                geom = stop_line_geom
                geom_type = getattr(stop_line_geom, "geom_type", "")
                if geom_type in ("LineString", "MultiLineString"):
                    geom = stop_line_geom.buffer(0.2)
                if not geom.intersection(adc_polygon).is_empty:
                    result.add(stop_sign_id)
            except Exception:
                continue
        return result

    def _current_lanelet_ids(self, localization: Odometry) -> Set[int]:
        map_parser = VectorMapParser.instance()
        if not hasattr(map_parser, "lanelet_map") or map_parser.lanelet_map is None:
            return set()
        point = BasicPoint2d(
            localization.pose.pose.position.x, localization.pose.pose.position.y
        )
        route_lanelet_ids: Set[int] = set()
        if hasattr(self, "oh") and self.oh:
            route_lanelet_ids = self.oh.get_route_lanelet_ids()

        ids: Set[int] = set()
        for lanelet in map_parser.lanelet_map.laneletLayer:
            if route_lanelet_ids and lanelet.id not in route_lanelet_ids:
                continue
            try:
                if inside(lanelet, point):
                    ids.add(int(lanelet.id))
            except Exception:
                continue

        # Fallback: if route filtering finds nothing, retry without route filter.
        if not ids and route_lanelet_ids:
            for lanelet in map_parser.lanelet_map.laneletLayer:
                try:
                    if inside(lanelet, point):
                        ids.add(int(lanelet.id))
                except Exception:
                    continue
        return ids

    def _get_lanes_controlled_by_regulatory_element(self, reg_elem_id: str) -> Set[int]:
        map_parser = VectorMapParser.instance()
        if not hasattr(map_parser, "lanelet_map") or map_parser.lanelet_map is None:
            return set()
        ids: Set[int] = set()
        for lanelet in map_parser.lanelet_map.laneletLayer:
            try:
                for reg_elem in lanelet.regulatoryElements:
                    if str(reg_elem.id) == str(reg_elem_id):
                        ids.add(int(lanelet.id))
                        break
            except Exception:
                continue
        return ids

    def parse_stop_sign_stop_line_on_map(self) -> None:
        self.stop_sign_stop_line_dict = {}
        self.stop_sign_id_to_lane_ids = {}
        self.stop_sign_id_to_event_key = {}
        self.event_key_to_stop_sign_ids = {}
        try:
            map_parser = VectorMapParser.instance()
            if not hasattr(map_parser, "lanelet_map") or map_parser.lanelet_map is None:
                return
        except Exception:
            return

        try:
            stop_sign_ids = map_parser.get_stop_signs()
        except Exception:
            return

        for ss_id in stop_sign_ids:
            try:
                stop_line = map_parser.get_stop_line_for_stop_sign(ss_id)
                if stop_line is not None:
                    stop_sign_id = str(ss_id)
                    self.stop_sign_stop_line_dict[stop_sign_id] = stop_line
                    self.stop_sign_id_to_lane_ids[stop_sign_id] = (
                        self._get_lanes_controlled_by_regulatory_element(stop_sign_id)
                    )
                    event_key = self._stop_line_event_key(stop_line) or f"stop_sign:{stop_sign_id}"
                    self.stop_sign_id_to_event_key[stop_sign_id] = event_key
                    if event_key not in self.event_key_to_stop_sign_ids:
                        self.event_key_to_stop_sign_ids[event_key] = set()
                    self.event_key_to_stop_sign_ids[event_key].add(stop_sign_id)
            except Exception:
                continue

    def get_result(self) -> List[Violation]:
        result: List[Violation] = []
        for event_key in sorted(self.violated_event_keys):
            features = self.violation_features.get(event_key)
            stop_sign_ids = sorted(self.event_key_to_stop_sign_ids.get(event_key, set()))
            representative_stop_sign_id = stop_sign_ids[0] if stop_sign_ids else event_key
            if features is None:
                if self.last_localization is not None:
                    features = self.get_basic_info_from_localization(self.last_localization)
                else:
                    features = self.get_dummy_basic_info()
                features["stop_sign_id"] = representative_stop_sign_id
                if len(stop_sign_ids) > 1:
                    features["stop_sign_ids"] = stop_sign_ids
            result.append(
                Violation("StopSignOracle", features, representative_stop_sign_id)
            )
        return result
