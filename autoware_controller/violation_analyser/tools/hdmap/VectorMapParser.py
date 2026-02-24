from typing import Any, Dict, List, Optional, Type, TypeVar
from dataclasses import dataclass
import xml.etree.ElementTree as ET
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


@dataclass
class StopSign:
    """Stop sign data structure."""
    id: str
    geometry: Point
    tags: Dict


class VectorMapParser:
    _instance = None
    lanelet_map: LaneletMap
    projector: MGRSProjector
    map_path: Optional[str] = None

    T = TypeVar('T')

    @classmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = cls.__new__(cls)
        return cls._instance

    @staticmethod
    def _attr_get(attrs: Any, key: str, default: Any = None) -> Any:
        try:
            if key in attrs:
                return attrs[key]
        except Exception:
            pass
        return default

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
        Return lanelets that belong to explicit map junctions.

        - return: Dict[lanelet id: junction_id]
        """
        explicit = self.get_attributes(key='junction_id', attribute_type=str)
        if explicit:
            return explicit

        # Fallback for maps without explicit junction_id tags (e.g., sample-map-planning):
        # treat signal-controlled lanelets as intersection lanelets.
        inferred: Dict[int, str] = {}
        try:
            lanes_by_signal = self.get_lanes_controlled_by_signal()
            for signal_id, lane_ids in lanes_by_signal.items():
                for lane_id in lane_ids:
                    if str(lane_id).isdigit():
                        inferred[int(lane_id)] = f"signal:{signal_id}"
        except Exception:
            pass

        if inferred:
            return inferred

        # Last-resort heuristic: lanelets with turn_direction are usually in junction areas.
        for lanelet in self.lanelet_map.laneletLayer:
            try:
                turn_direction = str(self._attr_get(lanelet.attributes, "turn_direction", ""))
            except Exception:
                turn_direction = ""
            if turn_direction:
                inferred[int(lanelet.id)] = "turn_direction"
        return inferred

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
                subtype = str(self._attr_get(reg_elem.attributes, "subtype", ""))
                if subtype in ['traffic_light', 'virtual_traffic_light']:
                    signal_ids.append(str(reg_elem.id))
            except (KeyError, AttributeError, TypeError, ValueError):
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
                    subtype = str(self._attr_get(reg_elem.attributes, "subtype", ""))
                    if subtype in ['traffic_light', 'virtual_traffic_light']:
                        signal_id = str(reg_elem.id)
                        if signal_id not in lanes_controlled_by_signal:
                            lanes_controlled_by_signal[signal_id] = []
                        if lane_id not in lanes_controlled_by_signal[signal_id]:
                            lanes_controlled_by_signal[signal_id].append(lane_id)
                except (KeyError, AttributeError, TypeError):
                    continue
        
        return lanes_controlled_by_signal

    def _get_regulatory_element_by_id(self, element_id: str):
        element_id_int = int(element_id) if str(element_id).isdigit() else None
        if element_id_int is None:
            return None
        try:
            return self.lanelet_map.regulatoryElementLayer.get(element_id_int)
        except Exception:
            return None

    @staticmethod
    def _linestring_from_any(obj: Any) -> Optional[LineString]:
        if obj is None:
            return None
        try:
            coords = []
            for pt in obj:
                if hasattr(pt, "x") and hasattr(pt, "y"):
                    coords.append((float(pt.x), float(pt.y)))
            if len(coords) >= 2:
                return LineString(coords)
        except Exception:
            pass
        return None

    @staticmethod
    def _point_from_any(obj: Any) -> Optional[Point]:
        if obj is None:
            return None
        if hasattr(obj, "x") and hasattr(obj, "y"):
            try:
                return Point(float(obj.x), float(obj.y))
            except Exception:
                return None
        return None

    def _get_lanes_controlled_by_regulatory_element(self, reg_elem_id: str) -> List[str]:
        lane_ids: List[str] = []
        for lanelet in self.lanelet_map.laneletLayer:
            for reg_elem in lanelet.regulatoryElements:
                try:
                    if str(reg_elem.id) == str(reg_elem_id):
                        lane_ids.append(str(lanelet.id))
                        break
                except Exception:
                    continue
        return lane_ids

    def _fallback_stop_line_from_controlled_lanes(self, reg_elem_id: str) -> Optional[LineString]:
        lane_ids = self._get_lanes_controlled_by_regulatory_element(reg_elem_id)
        for lane_id in lane_ids:
            try:
                lane = self.get_lane_by_id(lane_id)
                if lane.centerline and len(lane.centerline.coords) >= 2:
                    p0 = lane.centerline.coords[0]
                    p1 = lane.centerline.coords[1]
                    return LineString([p0, p1])
            except Exception:
                continue
        return None

    @staticmethod
    def _parse_local_xy_nodes(root) -> Dict[str, tuple]:
        nodes: Dict[str, tuple] = {}
        for node in root.findall("node"):
            node_id = node.attrib.get("id")
            if node_id is None:
                continue
            local_x = None
            local_y = None
            for tag in node.findall("tag"):
                k = tag.attrib.get("k")
                if k == "local_x":
                    local_x = tag.attrib.get("v")
                elif k == "local_y":
                    local_y = tag.attrib.get("v")
            if local_x is None or local_y is None:
                continue
            try:
                nodes[node_id] = (float(local_x), float(local_y))
            except Exception:
                continue
        return nodes

    def _extract_ref_line_from_osm_relation(self, reg_elem_id: str) -> Optional[LineString]:
        """
        Extract relation ref_line from the source OSM directly.
        This avoids false positives caused by lane-centerline fallback when
        lanelet2 regulatory member access is unavailable in Python bindings.
        """
        if not self.map_path:
            return None
        try:
            root = ET.parse(self.map_path).getroot()
        except Exception:
            return None

        nodes = self._parse_local_xy_nodes(root)
        if not nodes:
            return None

        way_by_id: Dict[str, List[tuple]] = {}
        for way in root.findall("way"):
            way_id = way.attrib.get("id")
            if way_id is None:
                continue
            coords: List[tuple] = []
            for nd in way.findall("nd"):
                ref = nd.attrib.get("ref")
                if ref in nodes:
                    coords.append(nodes[ref])
            if len(coords) >= 2:
                way_by_id[way_id] = coords

        relation = None
        for rel in root.findall("relation"):
            if rel.attrib.get("id") == str(reg_elem_id):
                relation = rel
                break
        if relation is None:
            return None

        ref_way_ids: List[str] = []
        for member in relation.findall("member"):
            if member.attrib.get("type") != "way":
                continue
            if member.attrib.get("role") in ("ref_line", "stop_line", "stopLine"):
                ref = member.attrib.get("ref")
                if ref:
                    ref_way_ids.append(ref)

        for way_id in ref_way_ids:
            coords = way_by_id.get(way_id)
            if coords and len(coords) >= 2:
                try:
                    return LineString(coords)
                except Exception:
                    continue
        return None

    def _extract_stop_line_for_regulatory_element(
        self, reg_elem_id: str, strict: bool = False
    ) -> Optional[LineString]:
        reg_elem = self._get_regulatory_element_by_id(reg_elem_id)
        if reg_elem is None:
            return None

        attr_names = ("stopLine", "stopLines", "refLines")
        if not strict:
            attr_names = attr_names + ("refers",)

        for attr_name in attr_names:
            if not hasattr(reg_elem, attr_name):
                continue
            try:
                value = getattr(reg_elem, attr_name)
            except Exception:
                continue
            if callable(value):
                try:
                    value = value()
                except Exception:
                    continue
            if value is None:
                continue
            if isinstance(value, (list, tuple)):
                for item in value:
                    line = self._linestring_from_any(item)
                    if line is not None:
                        return line
            else:
                line = self._linestring_from_any(value)
                if line is not None:
                    return line

        # Prefer explicit OSM ref_line/stop_line if lanelet2 relation member API is unavailable.
        osm_ref_line = self._extract_ref_line_from_osm_relation(reg_elem_id)
        if osm_ref_line is not None:
            return osm_ref_line

        # Strict mode is used for stop-signs: do not infer stop-lines from lane centerlines.
        if strict:
            return None

        return self._fallback_stop_line_from_controlled_lanes(reg_elem_id)

    def get_stop_line_for_signal(self, signal_id: str) -> Optional[LineString]:
        return self._extract_stop_line_for_regulatory_element(str(signal_id))

    def get_stop_signs(self) -> List[str]:
        stop_sign_ids: List[str] = []
        for reg_elem in self.lanelet_map.regulatoryElementLayer:
            try:
                subtype = str(self._attr_get(reg_elem.attributes, "subtype", "")).lower()
            except Exception:
                continue
            # Keep candidate set broad for map compatibility, then enforce explicit
            # stop-line extraction in get_stop_line_for_stop_sign(strict=True).
            if subtype in ("traffic_sign", "stop_sign", "all_way_stop"):
                stop_sign_ids.append(str(reg_elem.id))
        return stop_sign_ids

    def get_stop_sign_by_id(self, stop_sign_id: str) -> StopSign:
        reg_elem = self._get_regulatory_element_by_id(stop_sign_id)
        if reg_elem is None:
            raise ValueError(f"Stop sign {stop_sign_id} not found")

        geometry = None
        for attr_name in ("refers", "refLines"):
            if not hasattr(reg_elem, attr_name):
                continue
            try:
                refs = getattr(reg_elem, attr_name)
            except Exception:
                continue
            if callable(refs):
                try:
                    refs = refs()
                except Exception:
                    continue
            if refs is None:
                continue
            if isinstance(refs, (list, tuple)):
                for ref in refs:
                    point = self._point_from_any(ref)
                    if point is not None:
                        geometry = point
                        break
            else:
                geometry = self._point_from_any(refs)
            if geometry is not None:
                break

        if geometry is None:
            stop_line = self.get_stop_line_for_stop_sign(stop_sign_id)
            if stop_line is not None:
                try:
                    coords = list(stop_line.coords)
                    if coords:
                        geometry = Point(coords[0][0], coords[0][1])
                except Exception:
                    geometry = None
        if geometry is None:
            lanes = self._get_lanes_controlled_by_regulatory_element(str(stop_sign_id))
            for lane_id in lanes:
                try:
                    lane = self.get_lane_by_id(lane_id)
                    if lane.centerline and len(lane.centerline.coords) > 0:
                        first = lane.centerline.coords[0]
                        geometry = Point(first[0], first[1])
                        break
                except Exception:
                    continue
        if geometry is None:
            raise ValueError(f"Could not resolve geometry for stop sign {stop_sign_id}")

        tags: Dict[str, str] = {}
        try:
            for key in reg_elem.attributes:
                tags[key] = str(reg_elem.attributes[key])
        except Exception:
            pass
        return StopSign(id=str(stop_sign_id), geometry=geometry, tags=tags)

    def get_stop_line_for_stop_sign(self, stop_sign_id: str) -> Optional[LineString]:
        return self._extract_stop_line_for_regulatory_element(
            str(stop_sign_id), strict=True
        )
