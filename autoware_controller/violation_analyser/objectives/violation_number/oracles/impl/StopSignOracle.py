from typing import Dict, List, Optional, Set

from nav_msgs.msg import Odometry

from objectives.violation_number.oracles.OracleInterface import OracleInterface
from objectives.violation_number.oracles.Violation import Violation
from tools.autoware_tools.calculate_velocity import calculate_velocity
from tools.hdmap.VectorMapParser import VectorMapParser
from tools.utils import StopLineCrossings


class StopSignOracle(OracleInterface):
    """
    Stop-sign oracle.

    A violation is recorded when the ego vehicle's FRONT AXLE crosses a
    stop-sign stop line without having come to a complete stop on the approach
    to it.

    "Came to a complete stop" is LATCHED while the axle is within
    APPROACH_ZONE_M before the line, not searched for in a time window ending
    at the crossing. The window this replaces was 5 s, and a vehicle waiting
    longer than that at the sign -- for a gap, or behind another car -- had its
    stop fall out of the window and was reported for a stop it had made. Dwell
    time is exactly what a stop sign asks for, so it must not bound the check.

    See StopLineCrossings for why the reference point is the front axle and the
    event is a crossing: Autoware stops with `stop_margin: 0.0`, i.e. with the
    bumper ON the line, so a footprint test fires on every correct stop -- and
    fires at the moment the bumper arrives, which is BEFORE the vehicle has
    finished stopping, so no look-back could have found the stop either.
    """

    STOPPED_SPEED_MPS = 0.01
    # How far back from the line a complete stop still counts as stopping FOR
    # the sign. Generous on purpose: a vehicle held in a queue stops well short
    # and then creeps to the line without a second full stop, and reporting
    # that as a rolling stop is the more damaging error of the two.
    APPROACH_ZONE_M = 15.0

    last_localization: Optional[Odometry]
    stop_sign_stop_line_dict: Dict[str, object]
    crossings: Optional[StopLineCrossings]
    stop_sign_id_to_lane_ids: Dict[str, Set[int]]
    stop_sign_id_to_event_key: Dict[str, str]
    event_key_to_stop_sign_ids: Dict[str, Set[str]]
    stopped_on_approach: Set[str]
    violated_event_keys: Set[str]
    violation_features: Dict[str, dict]

    def __init__(self) -> None:
        self.last_localization = None
        self.stop_sign_stop_line_dict = {}
        self.crossings = None
        self.stop_sign_id_to_lane_ids = {}
        self.stop_sign_id_to_event_key = {}
        self.event_key_to_stop_sign_ids = {}
        self.stopped_on_approach = set()
        self.violated_event_keys = set()
        self.violation_features = {}
        self.parse_stop_sign_stop_line_on_map()

    def get_interested_topics(self) -> List[str]:
        return ["/localization/kinematic_state"]

    def on_new_message(self, topic: str, message, t):
        if topic != "/localization/kinematic_state":
            return
        self.last_localization = message
        if self.crossings is None:
            return

        offsets, crossed_ids = self.crossings.update(message)
        speed = calculate_velocity(message.twist.twist.linear)

        # Latch the stop first: a vehicle can stop and cross on consecutive
        # samples, and the stop is what excuses the crossing.
        if speed <= self.STOPPED_SPEED_MPS:
            for stop_sign_id, offset in offsets.items():
                if offset is None or not (-self.APPROACH_ZONE_M <= offset <= 0.0):
                    continue
                self.stopped_on_approach.add(self._event_key(stop_sign_id))

        for stop_sign_id in crossed_ids:
            if not self._is_on_route(stop_sign_id):
                continue
            event_key = self._event_key(stop_sign_id)
            if event_key in self.stopped_on_approach:
                continue
            if event_key in self.violated_event_keys:
                continue
            self.violated_event_keys.add(event_key)

            stop_sign_ids = sorted(self.event_key_to_stop_sign_ids.get(event_key, set()))
            representative_stop_sign_id = stop_sign_ids[0] if stop_sign_ids else event_key
            features = self.get_basic_info_from_localization(message)
            features["stop_sign_id"] = representative_stop_sign_id
            if len(stop_sign_ids) > 1:
                features["stop_sign_ids"] = stop_sign_ids
            self.violation_features[event_key] = features

    def _event_key(self, stop_sign_id: str) -> str:
        return self.stop_sign_id_to_event_key.get(stop_sign_id, f"stop_sign:{stop_sign_id}")

    def _is_on_route(self, stop_sign_id: str) -> bool:
        """Ignore stop signs that govern lanes the ego is not routed through.

        The lane the ego occupies is NOT used for this. The lanelets a stop
        sign controls begin at its stop line, so at the moment the axle crosses
        one the ego's pose is still a wheel_base short of them -- gating on the
        current lanelet would suppress every crossing. What confines a crossing
        to the right road is the stop line's own extent, which
        StopLineCrossings checks.
        """
        route_lanelet_ids: Set[int] = set()
        if getattr(self, "oh", None):
            route_lanelet_ids = self.oh.get_route_lanelet_ids()
        controlled_lane_ids = self.stop_sign_id_to_lane_ids.get(stop_sign_id, set())
        if not route_lanelet_ids or not controlled_lane_ids:
            return True
        return not controlled_lane_ids.isdisjoint(route_lanelet_ids)

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

    def _get_lanes_controlled_by_regulatory_element(self, reg_elem_id: str) -> Set[int]:
        map_parser = VectorMapParser.instance()
        if not hasattr(map_parser, "lanelet_map") or map_parser.lanelet_map is None:
            return set()
        # One shared pass over laneletLayer, cached on the parser, instead of
        # one walk of the whole layer per stop sign. See
        # VectorMapParser.get_lanelets_for_regulatory_element.
        return map_parser.get_lanelets_for_regulatory_element(reg_elem_id)

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
            stop_sign_ids = []

        for ss_id in stop_sign_ids:
            try:
                stop_line = map_parser.get_stop_line_for_stop_sign(ss_id)
                if stop_line is None:
                    continue
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
        self.crossings = StopLineCrossings(self.stop_sign_stop_line_dict)

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
