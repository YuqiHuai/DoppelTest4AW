from typing import Dict, List, Optional, Set
import math
from shapely.geometry import LineString, Polygon, Point
from shapely.ops import unary_union

from objectives.violation_number.oracles.OracleInterface import OracleInterface
from objectives.violation_number.oracles.Violation import Violation
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import TrafficLightGroupArray
from tools.utils import generate_adc_polygon, quaternion_2_heading
from tools.autoware_tools.calculate_velocity import calculate_velocity
from tools.hdmap.VectorMapParser import VectorMapParser
from config import AUTOWARE_VEHICLE_LENGTH, AUTOWARE_VEHICLE_back_edge_to_center

try:
    from lanelet2.core import BasicPoint2d
    from lanelet2.geometry import inside
    LANELET2_AVAILABLE = True
except ImportError:
    LANELET2_AVAILABLE = False
    BasicPoint2d = None
    inside = None


class TrafficSignalOracle(OracleInterface):
    """
    The Traffic Signal Oracle is responsible for checking if the ADS instance violates a red light
    signal. When the AV is facing a red light signal and the AV is already intersecting the stop line,
    it should not have positive speed because it should have stopped before the stop line.
    """

    last_localization: Optional[Odometry]
    last_traffic_signal_detection: Optional[TrafficLightGroupArray]
    traffic_signal_stop_line_string_dict: Dict[str, LineString]
    violated_at_traffic_signal_ids: Set[str]
    red_signal_states: Dict[str, str]
    violation_features: Dict[str, dict]
    stopline_to_signals: Dict[tuple, List[str]]

    def __init__(self) -> None:
        self.violated_at_traffic_signal_ids = set()
        self.last_localization = None
        self.last_traffic_signal_detection = None
        self.traffic_signal_stop_line_string_dict = dict()
        self.red_signal_states = {}
        self.violation_features = {}
        self.stopline_to_signals = {}
        self._initialize_map_parser()

    def _initialize_map_parser(self):
        """Initialize VectorMapParser and parse traffic signal stop lines."""
        try:
            map_parser = VectorMapParser.instance()
            if not hasattr(map_parser, 'lanelet_map') or map_parser.lanelet_map is None:
                return
            self.parse_traffic_signal_stop_line_string_on_map(map_parser)
        except Exception:
            pass

    def get_interested_topics(self):
        """Return topics of interest: localization and traffic signals."""
        return [
            '/localization/kinematic_state',
            '/perception/traffic_light_recognition/traffic_signals',
        ]

    def on_new_message(self, topic: str, message, t):
        """Process incoming messages and check for violations."""
        if topic == '/localization/kinematic_state':
            self.last_localization = message
            self._check_violation_with_cached_states()
        elif topic == '/perception/traffic_light_recognition/traffic_signals':
            self.last_traffic_signal_detection = message
            signal_states = self._update_red_signal_states()
            if self.last_localization is not None:
                self._check_violation(signal_states)

    def _update_red_signal_states(self):
        """Update cached red signal states from traffic signal messages."""
        if self.last_traffic_signal_detection is None:
            return {}
        
        groups = getattr(self.last_traffic_signal_detection, 'traffic_light_groups',
                        getattr(self.last_traffic_signal_detection, 'signal_groups',
                               getattr(self.last_traffic_signal_detection, 'groups', None)))
        
        if groups is None or len(groups) == 0:
            return self.red_signal_states if self.red_signal_states else {}

        signal_states = {}
        for group in groups:
            group_id = group.traffic_light_group_id
            if group_id is None:
                continue
            
            try:
                sig_id_int = int(group_id) if not isinstance(group_id, str) else int(group_id)
                sig_id_str = str(group_id)
            except (ValueError, TypeError):
                sig_id_int = None
                sig_id_str = str(group_id) if group_id is not None else None
            
            is_red = False
            for light in group.elements:
                if hasattr(light, 'color'):
                    color_val = light.color
                    if isinstance(color_val, int):
                        if color_val == 1:  # RED
                            is_red = True
                            break
                    else:
                        color_str = str(color_val)
                        if 'RED' in color_str.upper() or color_str.endswith('_RED') or 'RED' == color_str:
                            is_red = True
                            break
                        try:
                            if int(color_str) == 1:
                                is_red = True
                                break
                        except (ValueError, TypeError):
                            pass
            
            signal_states[sig_id_str] = 'RED' if is_red else 'NOT_RED'
            if sig_id_int is not None:
                signal_states[sig_id_int] = 'RED' if is_red else 'NOT_RED'
                signal_states[str(sig_id_int)] = 'RED' if is_red else 'NOT_RED'
            if group_id not in signal_states:
                signal_states[group_id] = 'RED' if is_red else 'NOT_RED'
        
        for key, value in signal_states.items():
            self.red_signal_states[key] = value
        
        return signal_states

    def _check_violation_with_cached_states(self):
        """Check for violations using cached red signal states."""
        if self.last_localization is None:
            return
        if self.red_signal_states:
            self._check_violation(self.red_signal_states)

    def _check_violation(self, signal_states: Dict):
        """Check if vehicle is violating a red light based on signal states."""
        crossing_traffic_signal_id = self.check_if_adc_intersecting_any_stop_lines()
        if crossing_traffic_signal_id is None:
            return

        speed = calculate_velocity(self.last_localization.twist.twist.linear) if self.last_localization else 0
        crossing_signal_str = str(crossing_traffic_signal_id)
        
        # Determine which signal to check (linked to current lanelet)
        signal_to_check = crossing_traffic_signal_id
        try:
            map_parser = VectorMapParser.instance()
            if map_parser and self.last_localization and LANELET2_AVAILABLE:
                vehicle_pos = self.last_localization.pose.pose.position
                point = BasicPoint2d(vehicle_pos.x, vehicle_pos.y)
                
                for lanelet in map_parser.lanelet_map.laneletLayer:
                    if inside(lanelet, point):
                        current_lanelet_id = str(lanelet.id)
                        lanes_controlled_by_signal = map_parser.get_lanes_controlled_by_signal()
                        
                        linked_signals = [sig_id for sig_id, lanes in lanes_controlled_by_signal.items() 
                                         if current_lanelet_id in lanes]
                        
                        if linked_signals:
                            if crossing_signal_str in linked_signals:
                                signal_to_check = crossing_traffic_signal_id
                            elif len(linked_signals) == 1:
                                signal_to_check = linked_signals[0]
                        break
        except Exception:
            pass
        
        # Check if signal is red
        all_signal_states = {}
        all_signal_states.update(signal_states)
        all_signal_states.update(self.red_signal_states)
        
        is_red_signal = False
        key_formats = [signal_to_check, str(signal_to_check)]
        try:
            key_formats.extend([int(signal_to_check), str(int(signal_to_check))])
        except (ValueError, TypeError):
            pass
        
        for key_format in key_formats:
            if key_format is not None and key_format in all_signal_states:
                if all_signal_states[key_format] == 'RED':
                    is_red_signal = True
                    break
        
        if not is_red_signal:
            return

        # Check if vehicle is stopped
        if speed <= 0.01:
            return
        
        # Violation detected
        violation_key = str(crossing_traffic_signal_id)
        if violation_key not in self.violated_at_traffic_signal_ids:
            if self.last_localization:
                features = self.get_basic_info_from_localization(self.last_localization)
                features['traffic_signal_id'] = violation_key
                self.violation_features[violation_key] = features
            else:
                features = self.get_dummy_basic_info()
                features['traffic_signal_id'] = violation_key
                self.violation_features[violation_key] = features
            
            self.violated_at_traffic_signal_ids.add(violation_key)

    def get_result(self) -> List[Violation]:
        """Returns a list of violations from this oracle."""
        result = list()
        for traffic_signal_id in self.violated_at_traffic_signal_ids:
            if traffic_signal_id in self.violation_features:
                features = self.violation_features[traffic_signal_id]
            else:
                if self.last_localization:
                    features = self.get_basic_info_from_localization(self.last_localization)
                    features['traffic_signal_id'] = traffic_signal_id
                else:
                    features = self.get_dummy_basic_info()
                    features['traffic_signal_id'] = traffic_signal_id
            
            violation = Violation('TrafficSignalOracle', features, traffic_signal_id)
            result.append(violation)
        return result

    def parse_traffic_signal_stop_line_string_on_map(self, map_parser) -> None:
        """Parse and store stop line for every traffic signal on the map."""
        if map_parser is None:
            return
            
        self.traffic_signal_stop_line_string_dict = dict()
        self.stopline_to_signals = {}
        
        try:
            traffic_signal_ids = map_parser.get_signals()
            lanes_controlled_by_signal = map_parser.get_lanes_controlled_by_signal()
            
            for ts_id in traffic_signal_ids:
                try:
                    traffic_signal_data = map_parser.get_signal_by_id(ts_id)
                    signal_point = traffic_signal_data.geometry
                    
                    stop_line_geom = None
                    stop_line = map_parser.get_stop_line_for_signal(ts_id)
                    
                    if stop_line is not None:
                        stop_line_geom = stop_line.buffer(0.2)
                        coords_key = tuple((round(x, 2), round(y, 2)) for x, y in stop_line.coords)
                        if coords_key not in self.stopline_to_signals:
                            self.stopline_to_signals[coords_key] = []
                        self.stopline_to_signals[coords_key].append(str(ts_id))
                    else:
                        controlled_lanes = lanes_controlled_by_signal.get(str(ts_id), [])
                        if controlled_lanes:
                            stop_line_points = []
                            for lane_id in controlled_lanes:
                                try:
                                    lane = map_parser.get_lane_by_id(lane_id)
                                    if lane.centerline and len(lane.centerline.coords) > 0:
                                        stop_line_points.append(Point(lane.centerline.coords[0]))
                                except Exception:
                                    continue
                            
                            if stop_line_points:
                                all_points = [signal_point] + stop_line_points
                                buffers = [pt.buffer(2.0) for pt in all_points]
                                stop_line_geom = unary_union(buffers) if len(buffers) > 1 else buffers[0]
                            else:
                                stop_line_geom = signal_point.buffer(3.0)
                        else:
                            stop_line_geom = signal_point.buffer(3.0)
                    
                    if stop_line_geom is not None:
                        self.traffic_signal_stop_line_string_dict[str(ts_id)] = stop_line_geom
                except Exception:
                    continue
        except Exception:
            pass

    def check_if_adc_intersecting_any_stop_lines(self) -> Optional[str]:
        """Check if the AV is intersecting any stop line."""
        if self.last_localization is None:
            return None

        adc_pose = self.last_localization.pose.pose
        adc_heading = quaternion_2_heading(adc_pose.orientation)
        adc_polygon_pts = generate_adc_polygon(adc_pose.position, adc_heading)
        adc_polygon = Polygon([[x.x, x.y] for x in adc_polygon_pts])
        
        # Calculate vehicle front edge position
        front_offset = AUTOWARE_VEHICLE_LENGTH - AUTOWARE_VEHICLE_back_edge_to_center
        front_x = adc_pose.position.x + front_offset * math.cos(adc_heading)
        front_y = adc_pose.position.y + front_offset * math.sin(adc_heading)
        vehicle_front = Point(front_x, front_y)
        vehicle_rear_axle = Point(adc_pose.position.x, adc_pose.position.y)

        for traffic_signal_id, stop_line_geom in self.traffic_signal_stop_line_string_dict.items():
            try:
                if hasattr(stop_line_geom, 'intersects'):
                    # Ensure line-like geometries are buffered for robust intersection
                    geom = stop_line_geom
                    if hasattr(stop_line_geom, "geom_type") and stop_line_geom.geom_type in ("LineString", "MultiLineString"):
                        geom = stop_line_geom.buffer(0.5)
                    intersection = geom.intersection(adc_polygon)
                    if not intersection.is_empty:
                        intersection_area = intersection.area if hasattr(intersection, 'area') else 0.0
                        intersection_length = intersection.length if hasattr(intersection, 'length') else 0.0
                        if intersection_area > 0.005 or intersection_length > 0.1:
                            return traffic_signal_id
                elif hasattr(stop_line_geom, 'distance'):
                    dist = stop_line_geom.distance(adc_polygon)
                    if dist == 0 or dist < 0.3:
                        return traffic_signal_id
            except Exception:
                continue

        return None

    def is_adc_completely_stopped(self) -> bool:
        """Check if the AV is currently stopped."""
        if self.last_localization is None:
            return False
        adc_velocity = calculate_velocity(self.last_localization.twist.twist.linear)
        return adc_velocity <= 0.01
