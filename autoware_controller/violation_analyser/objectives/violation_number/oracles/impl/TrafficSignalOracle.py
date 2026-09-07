import math
from typing import Dict, List, Optional, Set, Tuple

from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroupArray
from nav_msgs.msg import Odometry

from config import AUTOWARE_VEHICLE_WHEEL_BASE
from objectives.violation_number.oracles.OracleInterface import OracleInterface
from objectives.violation_number.oracles.Violation import Violation
from tools.autoware_tools.calculate_velocity import calculate_velocity
from tools.hdmap.VectorMapParser import VectorMapParser
from tools.utils import quaternion_2_heading

Frame = Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float], float]


class TrafficSignalOracle(OracleInterface):
    """
    Red-light violation oracle.

    A violation is recorded when the ego vehicle's FRONT AXLE crosses the stop
    line of a signal that is red at the moment of the crossing, while moving.

    Two things about that definition are deliberate, because the obvious
    alternatives both report a correctly stopping vehicle:

    - The reference point is the front axle, not the vehicle footprint.
      Autoware's traffic_light module runs with `stop_margin: 0.0`, so a
      CORRECT stop puts the front bumper on the stop line and holds it there.
      Any test anchored at the bumper -- such as intersecting the footprint
      with the line -- therefore fires on every properly executed stop at a
      red light. Measured on BorregasAve: the ego stopped with its pose 3.4-3.6
      m short of the line, bumper 0.2-0.3 m past it, front axle still 0.7-0.8 m
      short. The front axle is a wheel_base behind the bumper and stays clear.

    - The event is a CROSSING (from before the line to beyond it), not the
      state of being beyond it. A vehicle that entered on green and is still
      in the intersection when the signal turns red has not run the light, but
      it is `beyond` the line for as long as the red lasts.

    Event-level counting:
      - multiple signal IDs can share one physical stop line on map,
      - this oracle deduplicates such IDs into one violation event.
    """

    # A crossing at a standstill is not a crossing; this only guards against
    # localization jitter, since passing the line implies motion anyway.
    STOPPED_SPEED_MPS = 0.01
    # How far past either end of the stop line the crossing point may lie and
    # still count. A stop line spans its approach, so a vehicle passing outside
    # this window is not on the road the line governs.
    SPAN_TOLERANCE_M = 1.0

    last_localization: Optional[Odometry]
    last_traffic_signal_detection: Optional[TrafficLightGroupArray]
    red_signal_ids: Set[str]
    traffic_signal_stop_line_string_dict: Dict[str, object]
    stop_line_frames: Dict[str, Frame]
    stop_line_orientation: Dict[str, float]
    last_axle_offset: Dict[str, Optional[float]]
    signal_id_to_event_key: Dict[str, str]
    event_key_to_signal_ids: Dict[str, Set[str]]
    violated_event_keys: Set[str]
    violation_features: Dict[str, dict]

    def __init__(self) -> None:
        self.last_localization = None
        self.last_traffic_signal_detection = None
        self.red_signal_ids = set()
        self.traffic_signal_stop_line_string_dict = dict()
        self.stop_line_frames = {}
        self.stop_line_orientation = {}
        self.last_axle_offset = {}
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

    @staticmethod
    def _stop_line_frame(geom) -> Optional[Frame]:
        """Origin, tangent, normal and span of a stop line.

        The normal has no inherent direction -- which side is `beyond` depends
        on which way the vehicle drives -- so it is oriented per vehicle the
        first time the line is approached, in _axle_offset.
        """
        coords: List[tuple] = []
        geom_type = getattr(geom, "geom_type", "")
        if geom_type == "LineString":
            coords = list(geom.coords)
        elif geom_type == "MultiLineString":
            for line in getattr(geom, "geoms", []):
                coords.extend(list(line.coords))
        if len(coords) < 2:
            return None
        x0, y0 = float(coords[0][0]), float(coords[0][1])
        x1, y1 = float(coords[-1][0]), float(coords[-1][1])
        dx, dy = x1 - x0, y1 - y0
        span = math.hypot(dx, dy)
        if span < 1e-6:
            return None
        return (x0, y0), (dx / span, dy / span), (-dy / span, dx / span), span

    def _axle_offset(
        self, signal_id: str, frame: Frame, axle_x: float, axle_y: float,
        heading_x: float, heading_y: float,
    ) -> Optional[float]:
        """Signed distance from the stop line to the front axle, positive
        beyond it, or None when the axle is not within the line's own extent.
        """
        (x0, y0), (tx, ty), (nx, ny), span = frame
        rx, ry = axle_x - x0, axle_y - y0
        along = rx * tx + ry * ty
        if along < -self.SPAN_TOLERANCE_M or along > span + self.SPAN_TOLERANCE_M:
            return None
        # Fix the normal's direction once per line, from the heading the
        # vehicle first approached it with. Re-deriving it every sample would
        # flip the sign -- and so fake a crossing -- when a vehicle turns.
        orientation = self.stop_line_orientation.get(signal_id)
        if orientation is None:
            orientation = 1.0 if (nx * heading_x + ny * heading_y) > 0 else -1.0
            self.stop_line_orientation[signal_id] = orientation
        return (rx * nx + ry * ny) * orientation

    def _check_violation(self) -> None:
        if self.last_localization is None:
            return
        if not self.stop_line_frames:
            return

        pose = self.last_localization.pose.pose
        heading = quaternion_2_heading(pose.orientation)
        heading_x, heading_y = math.cos(heading), math.sin(heading)
        axle_x = pose.position.x + AUTOWARE_VEHICLE_WHEEL_BASE * heading_x
        axle_y = pose.position.y + AUTOWARE_VEHICLE_WHEEL_BASE * heading_y
        speed = calculate_velocity(self.last_localization.twist.twist.linear)

        crossed_ids: List[str] = []
        for signal_id, frame in self.stop_line_frames.items():
            offset = self._axle_offset(
                signal_id, frame, axle_x, axle_y, heading_x, heading_y
            )
            previous = self.last_axle_offset.get(signal_id)
            self.last_axle_offset[signal_id] = offset
            # Both samples must be inside the line's extent: without a `before`
            # there is no crossing to speak of, only a vehicle that was already
            # past the line when the recording -- or the red -- began.
            if offset is None or previous is None:
                continue
            if not (previous <= 0.0 < offset):
                continue
            if signal_id not in self.red_signal_ids:
                continue
            if speed <= self.STOPPED_SPEED_MPS:
                continue
            crossed_ids.append(signal_id)

        if not crossed_ids:
            return

        crossed_event_keys: Set[str] = set()
        for signal_id in crossed_ids:
            event_key = self.signal_id_to_event_key.get(signal_id, f"signal:{signal_id}")
            crossed_event_keys.add(event_key)

        for event_key in crossed_event_keys:
            if event_key in self.violated_event_keys:
                continue
            signal_ids = sorted(self.event_key_to_signal_ids.get(event_key, set()))
            if not signal_ids:
                signal_ids = sorted(
                    [
                        sid
                        for sid in crossed_ids
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

    def parse_traffic_signal_stop_line_string_on_map(self) -> None:
        self.traffic_signal_stop_line_string_dict = dict()
        self.stop_line_frames = {}
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
                if stop_line is None:
                    continue
                frame = self._stop_line_frame(stop_line)
                if frame is None:
                    continue
                signal_id = str(ts_id)
                self.traffic_signal_stop_line_string_dict[signal_id] = stop_line
                self.stop_line_frames[signal_id] = frame
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
