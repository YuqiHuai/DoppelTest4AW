import os
import pathlib

ROS_NODE_NAME_SERVER = "perception_server_node"
UVICORN_HOST = "0.0.0.0"
UVICORN_PORT = 5002
ROS_DETECTION_TOPIC = "/perception/object_recognition/detection/objects"
ROS_TRAFFIC_SIGNAL_TOPIC = "/perception/traffic_light_recognition/traffic_signals"

ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", "1")
os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID

VIOLATION_MAP_NAME = "sample-map-planning"
REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_MAP_PATH = REPO_ROOT / "autoware_map" / "sample-map-planning"
AUTOWARE_LAUNCH_PACKAGE = "autoware_launch"
AUTOWARE_LAUNCH_FILE = "planning_simulator.launch.xml"
DEFAULT_VEHICLE_MODEL = "sample_vehicle"
DEFAULT_SENSOR_MODEL = "sample_sensor_kit"


def _resolve_log_root() -> pathlib.Path:
    default_root = REPO_ROOT / "log"
    if ROS_DOMAIN_ID:
        default_root = REPO_ROOT / f"container_{ROS_DOMAIN_ID}" / "log"
    root = pathlib.Path(os.environ.get("RECEIVER_LOG_ROOT", str(default_root)))
    if not root.is_absolute():
        root = (REPO_ROOT / root).resolve()
    return root


LOG_ROOT = _resolve_log_root()
AUTOWARE_LOG_DIR = LOG_ROOT / "autoware_logs"
RECORD_LOG_DIR = LOG_ROOT / "record_log"
ROS_LOG_DIR = LOG_ROOT / "res_log"
