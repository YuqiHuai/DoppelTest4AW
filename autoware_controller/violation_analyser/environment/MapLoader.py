import xml.etree.ElementTree as ET
import lanelet2
from lanelet2.core import LaneletMap
from lanelet2.io import Origin
from config import ADS_MAP_DIR
from tools.hdmap.VectorMapParser import VectorMapParser
try:
    from autoware_lanelet2_extension_python.projection import MGRSProjector
except Exception:
    MGRSProjector = None
try:
    from lanelet2.projection import UTMProjector  # type: ignore
except Exception:
    UTMProjector = None
try:
    from lanelet2.projection import UtmProjector  # type: ignore
except Exception:
    UtmProjector = None

class MapLoader:

    def __init__(self, map_name):
        self.hd_map_path = f'{ADS_MAP_DIR}/{map_name}/lanelet2_map.osm'

        origin = self.get_first_reference_origin(self.hd_map_path)
        if MGRSProjector is None:
            projector_cls = UTMProjector or UtmProjector
            if projector_cls is None:
                raise ImportError(
                    "No lanelet2 projector available (MGRSProjector/UTMProjector)."
                )
            self.projector = projector_cls(origin)
        else:
            self.projector = MGRSProjector(origin)
        
        map_parser = VectorMapParser.instance()
        map_parser.lanelet_map = self.load_map()
        map_parser.projector = self.projector
        map_parser.map_path = self.hd_map_path
        self.map_instance = map_parser

    def load_map(self) -> LaneletMap:
        self.lanelet_map = lanelet2.io.load(self.hd_map_path, self.projector)
        return self.lanelet_map

    def get_first_reference_origin(self, hd_map_path: str) -> Origin:
        with open(hd_map_path, 'r') as file:
            xml_data = file.read()
        
        root = ET.fromstring(xml_data)
        first_node = root.find('.//node')

        lat = float(first_node.get('lat'))
        lon = float(first_node.get('lon'))
        return Origin(lat, lon, 0)
