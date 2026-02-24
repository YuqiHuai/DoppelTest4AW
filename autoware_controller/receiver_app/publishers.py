import math
import time
import uuid
import os
import queue
import threading
from typing import Any, Dict
from typing import List, Optional, Tuple

from builtin_interfaces.msg import Time
from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjects,
    ObjectClassification,
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)
from geometry_msgs.msg import (
    Point,
    Point32,
    Pose,
    Quaternion,
    Twist,
    Vector3,
)
from std_msgs.msg import Header as RosStdHeader
from unique_identifier_msgs.msg import UUID

from modules.localization.proto.localization_pb2 import LocalizationEstimate

from .state import server_globals

_STALE_OBJECT_SEC = 5.0


def _ensure_perception_worker():
    if server_globals.get("perception_queue") is not None:
        return

    qsize = int(server_globals.get("perception_queue_size") or 200)
    q = queue.Queue(maxsize=qsize)
    server_globals["perception_queue"] = q

    def _worker():
        while True:
            item = q.get()
            if item is None:
                break
            request_bytes, source_id = item
            try:
                loc_estimate = LocalizationEstimate()
                loc_estimate.ParseFromString(request_bytes)
                _update_and_publish(loc_estimate, source_id=source_id)
            except Exception as exc:
                node = server_globals.get("node")
                if node:
                    node.get_logger().error(f"Error processing perception request: {exc}")
            finally:
                q.task_done()

    worker = threading.Thread(target=_worker, daemon=True)
    server_globals["perception_worker"] = worker
    worker.start()


def _build_pedestrian_detected_object(
    position: Tuple[float, float, float],
    orientation: Tuple[float, float, float, float],
    speed_mps: float,
    pedestrian_id: Optional[str] = None,
) -> DetectedObject:
    ped_diam = 0.6
    ped_hgt = 1.7

    det = DetectedObject()
    det.existence_probability = 1.0

    cls = ObjectClassification()
    cls.label = 7
    cls.probability = 1.0
    det.classification = [cls]

    det.kinematics.pose_with_covariance.pose = Pose(
        position=Point(x=position[0], y=position[1], z=position[2]),
        orientation=Quaternion(
            x=orientation[0],
            y=orientation[1],
            z=orientation[2],
            w=orientation[3],
        ),
    )

    qx, qy, qz, qw = orientation
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    speed = float(speed_mps)
    det.kinematics.twist_with_covariance.twist = Twist(
        linear=Vector3(
            x=float(speed * math.cos(yaw)),
            y=float(speed * math.sin(yaw)),
            z=0.0,
        ),
        angular=Vector3(x=0.0, y=0.0, z=0.0),
    )
    det.kinematics.twist_with_covariance.covariance = [0.0] * 36
    det.kinematics.has_twist = False
    det.kinematics.has_twist_covariance = False

    det.shape.type = det.shape.CYLINDER
    det.shape.dimensions.x = ped_diam
    det.shape.dimensions.y = ped_diam
    det.shape.dimensions.z = ped_hgt

    if pedestrian_id:
        try:
            uuid_bytes = uuid.UUID(pedestrian_id).bytes
            det.object_id = UUID(uuid=list(uuid_bytes))
        except (ValueError, AttributeError):
            pass

    return det


async def process_perception_request_async(request_bytes: bytes, source_id: Optional[str] = None):
    node = server_globals.get("node")
    if not node:
        print(
            "[perception_server_node] Error: ROS node not initialized in process_perception_request_async."
        )
        return

    _ensure_perception_worker()
    q = server_globals.get("perception_queue")
    if not q:
        return
    try:
        q.put_nowait((request_bytes, source_id))
    except queue.Full:
        # Drop oldest to avoid backlog/bursty publish.
        try:
            _ = q.get_nowait()
            q.task_done()
        except Exception:
            pass
        try:
            q.put_nowait((request_bytes, source_id))
        except Exception:
            pass


def _build_detected_object(loc: LocalizationEstimate) -> DetectedObject:
    node = server_globals.get("node")
    detection_publisher = server_globals.get("detection_publisher")
    if not node or not detection_publisher:
        return

    veh_len = 2.79 + 1.0 + 1.1
    veh_wid = 1.64 + 0.128 + 0.128
    veh_hgt = 2.5
    center_offset_m = 1.345

    p = loc.pose.position
    q = loc.pose.orientation
    lv = loc.pose.linear_velocity
    av = loc.pose.angular_velocity

    det = DetectedObject()
    det.existence_probability = 1.0

    cls = ObjectClassification()
    cls.label = getattr(ObjectClassification, "VEHICLE", 1)
    cls.probability = 0.999
    det.classification = [cls]

    det.kinematics.pose_with_covariance.pose = Pose(
        position=Point(x=float(p.x), y=float(p.y), z=float(p.z)),
        orientation=Quaternion(
            x=float(q.qx if hasattr(q, "qx") else q.x),
            y=float(q.qy if hasattr(q, "qy") else q.y),
            z=float(q.qz if hasattr(q, "qz") else q.z),
            w=float(q.qw if hasattr(q, "qw") else q.w),
        ),
    )

    pose_cov = [0.0] * 36
    pose_cov[0] = pose_cov[7] = 0.01
    pose_cov[14] = 0.25
    pose_cov[35] = (5.0 * 3.14159265 / 180.0) ** 2
    det.kinematics.pose_with_covariance.covariance = pose_cov
    det.kinematics.has_position_covariance = True
    det.kinematics.orientation_availability = 1

    pose = det.kinematics.pose_with_covariance.pose
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    pose.position.x += center_offset_m * math.cos(yaw)
    pose.position.y += center_offset_m * math.sin(yaw)

    det.kinematics.twist_with_covariance.twist = Twist(
        linear=Vector3(x=float(lv.x), y=float(lv.y), z=float(lv.z)),
        angular=Vector3(x=float(av.x), y=float(av.y), z=float(av.z)),
    )
    tw_cov = [0.0] * 36
    tw_cov[0] = tw_cov[7] = 0.09
    tw_cov[14] = 0.25
    det.kinematics.twist_with_covariance.covariance = tw_cov
    det.kinematics.has_twist = True
    det.kinematics.has_twist_covariance = True

    det.shape.type = det.shape.BOUNDING_BOX
    det.shape.dimensions.x = veh_len
    det.shape.dimensions.y = veh_wid
    det.shape.dimensions.z = veh_hgt
    half_x = 0.5 * veh_len
    half_y = 0.5 * veh_wid
    det.shape.footprint.points = [
        Point32(x=half_x, y=half_y, z=0.0),
        Point32(x=half_x, y=-half_y, z=0.0),
        Point32(x=-half_x, y=-half_y, z=0.0),
        Point32(x=-half_x, y=half_y, z=0.0),
    ]

    # No object_id field in this Autoware message type; skip assigning IDs.

    return det


def _update_and_publish(loc: LocalizationEstimate, source_id: Optional[str] = None):
    node = server_globals.get("node")
    detection_publisher = server_globals.get("detection_publisher")
    if not node or not detection_publisher:
        return

    cache = server_globals.setdefault("perception_cache", {})
    lock = server_globals.setdefault("perception_cache_lock", threading.Lock())

    key = source_id or "unknown"
    det = _build_detected_object(loc)

    now = time.time()
    with lock:
        cache[key] = (now, det)
        # Drop stale entries
        stale_keys = [k for k, (ts, _d) in cache.items() if (now - ts) > _STALE_OBJECT_SEC]
        for k in stale_keys:
            cache.pop(k, None)
        objects = [d for _ts, d in cache.values()]

    out = DetectedObjects()
    out.header = RosStdHeader()
    out.header.stamp = Time(sec=int(now), nanosec=int((now - int(now)) * 1e9))
    out.header.frame_id = "map"
    out.objects = objects
    detection_publisher.publish(out)


def update_pedestrians_and_publish(pedestrians: List[Dict[str, Any]]):
    node = server_globals.get("node")
    detection_publisher = server_globals.get("detection_publisher")
    if not node or not detection_publisher:
        return

    cache = server_globals.setdefault("perception_cache", {})
    lock = server_globals.setdefault("perception_cache_lock", threading.Lock())

    now = time.time()
    with lock:
        for idx, ped in enumerate(pedestrians):
            ped_id = ped.get("pedestrian_id") or f"ped_{idx}"
            det = _build_pedestrian_detected_object(
                position=tuple(ped["position"]),
                orientation=tuple(ped.get("orientation") or [0.0, 0.0, 0.0, 1.0]),
                speed_mps=float(ped.get("speed") or 0.0),
                pedestrian_id=ped_id,
            )
            cache[f"ped:{ped_id}"] = (now, det)

        # Drop stale entries
        stale_keys = [k for k, (ts, _d) in cache.items() if (now - ts) > _STALE_OBJECT_SEC]
        for k in stale_keys:
            cache.pop(k, None)
        objects = [d for _ts, d in cache.values()]

    out = DetectedObjects()
    out.header = RosStdHeader()
    out.header.stamp = Time(sec=int(now), nanosec=int((now - int(now)) * 1e9))
    out.header.frame_id = "map"
    out.objects = objects
    detection_publisher.publish(out)


def publish_pedestrain(
    loc: LocalizationEstimate,
    speed_mps: float = 0.0,
    pedestrian_id: Optional[str] = None,
):
    p = loc.pose.position
    q = loc.pose.orientation
    update_pedestrians_and_publish(
        [
            {
                "pedestrian_id": pedestrian_id or "ped_single",
                "position": [float(p.x), float(p.y), float(p.z)],
                "orientation": [
                    float(q.qx if hasattr(q, "qx") else q.x),
                    float(q.qy if hasattr(q, "qy") else q.y),
                    float(q.qz if hasattr(q, "qz") else q.z),
                    float(q.qw if hasattr(q, "qw") else q.w),
                ],
                "speed": float(speed_mps),
            }
        ]
    )


def _publish_traffic_signal():
    node = server_globals.get("node")
    publisher = server_globals.get("traffic_signal_publisher")
    if not node or not publisher:
        return

    state = server_globals["traffic_signal_state"]
    msg = TrafficLightGroupArray()
    signal = TrafficLightGroup()
    signal.traffic_light_group_id = int(state["map_primitive_id"])

    element = TrafficLightElement()
    element.color = int(state["color"])
    element.shape = int(state["shape"])
    element.status = int(state["status"])
    element.confidence = float(state["confidence"])

    signal.elements.append(element)
    msg.traffic_light_groups.append(signal)
    now = time.time()
    msg.stamp = Time(sec=int(now), nanosec=int((now - int(now)) * 1e9))
    publisher.publish(msg)


def _publish_traffic_signals(signals: List[dict]):
    node = server_globals.get("node")
    publisher = server_globals.get("traffic_signal_publisher")
    if not node or not publisher:
        return

    msg = TrafficLightGroupArray()
    for signal_state in signals:
        signal = TrafficLightGroup()
        signal.traffic_light_group_id = int(signal_state["map_primitive_id"])

        element = TrafficLightElement()
        element.color = int(signal_state["color"])
        element.shape = int(signal_state["shape"])
        element.status = int(signal_state["status"])
        element.confidence = float(signal_state["confidence"])
        signal.elements.append(element)
        msg.traffic_light_groups.append(signal)

    now = time.time()
    msg.stamp = Time(sec=int(now), nanosec=int((now - int(now)) * 1e9))
    publisher.publish(msg)
