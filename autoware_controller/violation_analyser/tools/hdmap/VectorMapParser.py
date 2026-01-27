from typing import List, Dict, Type, TypeVar, Optional
from dataclasses import dataclass
from lanelet2.core import Lanelet, LaneletMap, RegulatoryElement
from shapely.geometry import Point, LineString
try:
    from autoware_lanelet2_extension_python.projection import MGRSProjector
except Exception:
    MGRSProjector = None
from tools.utils import construct_lane_boundary_linestring


@dataclass
class Signal:
    """Traffic signal data structure."""
    id: str
    geometry: Point
    tags: Dict


@dataclass
class Lane:
    """Lane data structure."""
    id: str
    centerline: LineString
    left_boundary: Optional[LineString] = None
    right_boundary: Optional[LineString] = None
    tags: Optional[Dict] = None


class VectorMapParser:
    _instance = None
    lanelet_map: LaneletMap
    projector: MGRSProjector

    T = TypeVar('T')

    @classmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = cls.__new__(cls)
        return cls._instance

    def __init__(self):
        raise RuntimeError('Call instance() instead')
      
    def get_attributes(self, key: str, attribute_type: Type[T]) -> Dict[int, T]:
        """
        - attribute_key: ['location', 'one_way', 'participant:vehicle', 'speed_limit', 'subtype', 'turn_direction', 'type']
        """
        result = {}
        for lanelet in self.lanelet_map.laneletLayer:
            if key in lanelet.attributes:
                result[lanelet.id] = attribute_type(lanelet.attributes[key])
        return result

    def get_lanelets(self, identifiers: List[int]) -> List[Lanelet]:
        return [lanelet for lanelet in self.lanelet_map.laneletLayer if lanelet.id in identifiers]

    def get_all_intersections(self) -> Dict[int, str]:
        """
        - return: Dict[lanelet id: turn_direction type]
        """
        return self.get_attributes(key='turn_direction', attribute_type=str)

    def get_lane_boundaries(self) -> dict:
        boundaries = dict()
        for lane in self.lanelet_map.laneletLayer:
            lane_id = lane.id
            l, r = construct_lane_boundary_linestring(lane)
            boundaries[f'{lane_id}_L'] = l
            boundaries[f'{lane_id}_R'] = r
        return boundaries

    def get_signals(self) -> List[str]:
        """
        Get a list of all traffic signal IDs on the HD Map.
        
        :returns: list of signal IDs (as strings)
        """
        signal_ids = []
        for reg_elem in self.lanelet_map.regulatoryElementLayer:
            try:
                # Check if it's a traffic light regulatory element
                subtype = reg_elem.attributes.get('subtype', '')
                if subtype in ['traffic_light', 'virtual_traffic_light']:
                    signal_ids.append(str(reg_elem.id))
            except (KeyError, AttributeError, TypeError):
                continue
        return signal_ids

    def get_signal_by_id(self, signal_id: str) -> Signal:
        """
        Get a traffic signal object by ID.
        
        :param signal_id: Signal ID (as string)
        :returns: Signal object with id and geometry (Point)
        """
        signal_id_int = int(signal_id) if signal_id.isdigit() else None
        if signal_id_int is None:
            raise ValueError(f"Invalid signal ID: {signal_id}")
        
        # Find the regulatory element
        reg_elem = self.lanelet_map.regulatoryElementLayer.get(signal_id_int)
        if reg_elem is None:
            raise ValueError(f"Signal {signal_id} not found")
        
        # Get signal position
        signal_point = None
        try:
            # Try to get position from trafficLights
            if hasattr(reg_elem, 'trafficLights') and reg_elem.trafficLights:
                for tl in reg_elem.trafficLights:
                    if hasattr(tl, 'x') and hasattr(tl, 'y'):
                        signal_point = Point(tl.x, tl.y)
                        break
        except (AttributeError, TypeError):
            pass
        
        # Fallback: try to get position from refers or ref_line
        if signal_point is None:
            try:
                if hasattr(reg_elem, 'refers') and reg_elem.refers:
                    for ref in reg_elem.refers:
                        if hasattr(ref, 'x') and hasattr(ref, 'y'):
                            signal_point = Point(ref.x, ref.y)
                            break
            except (AttributeError, TypeError):
                pass
        
        # Fallback: use center of first lanelet that references this signal
        if signal_point is None:
            for lanelet in self.lanelet_map.laneletLayer:
                if reg_elem in lanelet.regulatoryElements:
                    # Use the center of the lanelet
                    centerline = lanelet.centerline
                    if centerline and len(centerline) > 0:
                        # Get midpoint of centerline
                        mid_idx = len(centerline) // 2
                        pt = centerline[mid_idx]
                        signal_point = Point(pt.x, pt.y)
                        break
        
        if signal_point is None:
            raise ValueError(f"Could not determine position for signal {signal_id}")
        
        # Get tags/attributes
        tags = {}
        try:
            if hasattr(reg_elem, 'attributes'):
                for key in reg_elem.attributes:
                    try:
                        tags[key] = str(reg_elem.attributes[key])
                    except (KeyError, TypeError):
                        pass
        except (AttributeError, TypeError):
            pass
        
        return Signal(id=signal_id, geometry=signal_point, tags=tags)

    def get_lane_by_id(self, lane_id: str) -> Lane:
        """
        Get a lane object by ID.
        
        :param lane_id: Lane ID (as string or int)
        :returns: Lane object with id, centerline (LineString), and other properties
        """
        lane_id_int = int(lane_id) if str(lane_id).isdigit() else None
        if lane_id_int is None:
            raise ValueError(f"Invalid lane ID: {lane_id}")
        
        lanelet = self.lanelet_map.laneletLayer.get(lane_id_int)
        if lanelet is None:
            raise ValueError(f"Lane {lane_id} not found")
        
        # Get centerline
        centerline_pts = []
        if lanelet.centerline:
            for pt in lanelet.centerline:
                centerline_pts.append((pt.x, pt.y))
        centerline = LineString(centerline_pts) if centerline_pts else None
        
        # Get boundaries
        left_boundary = None
        right_boundary = None
        try:
            if lanelet.leftBound:
                left_pts = [(pt.x, pt.y) for pt in lanelet.leftBound]
                left_boundary = LineString(left_pts) if left_pts else None
            if lanelet.rightBound:
                right_pts = [(pt.x, pt.y) for pt in lanelet.rightBound]
                right_boundary = LineString(right_pts) if right_pts else None
        except (AttributeError, TypeError):
            pass
        
        # Get tags/attributes
        tags = {}
        try:
            if hasattr(lanelet, 'attributes'):
                for key in lanelet.attributes:
                    try:
                        tags[key] = str(lanelet.attributes[key])
                    except (KeyError, TypeError):
                        pass
        except (AttributeError, TypeError):
            pass
        
        return Lane(
            id=str(lane_id),
            centerline=centerline,
            left_boundary=left_boundary,
            right_boundary=right_boundary,
            tags=tags
        )

    def get_lanes_controlled_by_signal(self) -> Dict[str, List[str]]:
        """
        Get a mapping of signal IDs to the list of lane IDs they control.
        
        :returns: Dictionary mapping signal_id (str) to list of lane_ids (str)
        """
        lanes_controlled_by_signal = {}
        
        # Iterate through all lanelets
        for lanelet in self.lanelet_map.laneletLayer:
            lane_id = str(lanelet.id)
            
            # Check regulatory elements for traffic lights
            for reg_elem in lanelet.regulatoryElements:
                try:
                    subtype = reg_elem.attributes.get('subtype', '')
                    if subtype in ['traffic_light', 'virtual_traffic_light']:
                        signal_id = str(reg_elem.id)
                        if signal_id not in lanes_controlled_by_signal:
                            lanes_controlled_by_signal[signal_id] = []
                        if lane_id not in lanes_controlled_by_signal[signal_id]:
                            lanes_controlled_by_signal[signal_id].append(lane_id)
                except (KeyError, AttributeError, TypeError):
                    continue
        
        return lanes_controlled_by_signal
