import os
import pathlib

ROS_NODE_NAME_SERVER = "perception_server_node"
UVICORN_HOST = "0.0.0.0"
UVICORN_PORT = 5002
ROS_DETECTION_TOPIC = "/perception/object_recognition/detection/objects"
ROS_TRAFFIC_SIGNAL_TOPIC = "/perception/traffic_light_recognition/traffic_signals"
ROS_STATE_TOPIC = "/autoware/state"

ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", "1")
os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID

VIOLATION_MAP_NAME = "BorregasAve"
REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_MAP_PATH = REPO_ROOT / "autoware_map" / "BorregasAve"
AUTOWARE_LAUNCH_PACKAGE = "autoware_launch"
AUTOWARE_LAUNCH_FILE = "planning_simulator.launch.xml"
DEFAULT_VEHICLE_MODEL = "sample_vehicle"
DEFAULT_SENSOR_MODEL = "sample_sensor_kit"

#: The five behavior_velocity modules ``default_preset.yaml`` ships OFF that
#: this harness forces ON, so the stack under test is the same one Mozart4AW
#: drives.
#:
#: Without them the comparison is between configurations rather than
#: techniques. The preset disables eleven modules; a stock launch therefore
#: never constructs their managers, advertises no factor publisher, and no
#: obstacle, route or map can make them act -- so a zero in RQ1 means "the code
#: was absent", not "the search did not provoke it", and the two are the thing
#: RQ1 exists to tell apart. Measured before this change: DoppelTest produced
#: no ``merge_from_private`` factor in any run, on maps carrying 13 and 55
#: private-location lanelets, while Mozart reached it in 113 scenarios.
#:
#: These are Mozart's five (``mozart_autoware/receiver/config.py``), not the
#: preset's eleven. scenoRITA enables all eleven via ALL_MODULES=1, so the
#: three harnesses are 5/5/11 rather than equal; the six left off here are the
#: six outside RQ1's behaviour scope, which is the same line
#: ``research_questions/rq2/scope.yaml`` rule 2 draws for the coverage
#: denominator.
FORCED_MODULE_FLAGS = (
    "launch_merge_from_private_module",
    "launch_roundabout_module",
    "launch_occlusion_spot_module",
)

#: The other two, which their launch flags CANNOT turn on.
#:
#: ``behavior_planning.launch.xml`` appends ``experimental::NoDrivableLane\
#: ModulePlugin`` and ``experimental::SpeedBumpModulePlugin``, but both
#: packages export those classes in the OUTER namespace, so the flag asks
#: pluginlib for a name that does not exist and the module never loads.
#: ``behavior_velocity_planner_launch_modules`` defaults to the open bracket
#: ``"["``, so seeding it with the real class names prepends them and the
#: launch chain appends everything else as usual. Their flags stay off, which
#: is what keeps the non-existent experimental names out of the list.
EXTRA_LAUNCH_MODULES = (
    "autoware::behavior_velocity_planner::NoDrivableLaneModulePlugin",
    "autoware::behavior_velocity_planner::SpeedBumpModulePlugin",
)


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
