import math
import pathlib
from typing import Optional

from autoware_perception_msgs.msg import DetectedObject

from violation_analyser.tools.autoware_tools.rosbag_reader import ROSBagReader


def _object_is_vehicle(det_obj: DetectedObject) -> bool:
    if not det_obj.classification:
        return False
    return any(cls.label in {1, 2, 3, 4, 5} for cls in det_obj.classification)


def calculate_min_distance_from_record(record_path: pathlib.Path) -> Optional[float]:
    """
    Scan the rosbag and compute the smallest planar distance between the ego polygon
    and any perceived vehicle obstacle polygon (matching DoppelTest behavior).

    Only considers vehicles (excludes pedestrians) to match DoppelTest's min_distance
    calculation which is vehicle-to-vehicle only.
    """
    from violation_analyser.tools.utils import (
        generate_adc_polygon,
        obstacle_to_polygon,
        quaternion_2_heading,
    )
    from shapely.geometry import Polygon

    reader = ROSBagReader(str(record_path))
    localization_topic = "/localization/kinematic_state"
    perception_topic = "/perception/object_recognition/objects"
    last_localization_msg = None
    min_distance = math.inf

    for topic, data, _ in reader.read_messages():
        if topic == localization_topic:
            last_localization_msg = reader.deserialize_msg(data, topic)
        elif topic == perception_topic and last_localization_msg is not None:
            detected = reader.deserialize_msg(data, topic)

            ego_pose = last_localization_msg.pose.pose
            ego_heading = quaternion_2_heading(ego_pose.orientation)
            ego_polygon_pts = generate_adc_polygon(ego_pose.position, ego_heading)
            ego_polygon = Polygon([[pt.x, pt.y] for pt in ego_polygon_pts])

            for obj in getattr(detected, "objects", []):
                cls = getattr(obj, "classification", None)
                if not cls:
                    continue
                label = getattr(cls[0], "label", None) if cls else None
                if label not in {1, 2, 3, 4, 5}:
                    continue

                obs_polygon = obstacle_to_polygon(obj)
                dist = ego_polygon.distance(obs_polygon)
                if dist < min_distance:
                    min_distance = dist

    if math.isinf(min_distance):
        return None
    return min_distance
