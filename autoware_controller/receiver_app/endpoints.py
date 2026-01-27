import asyncio
import os
import pathlib
import signal
import subprocess
import time

import rclpy
from autoware_adapi_v1_msgs.srv import (
    ChangeOperationMode,
    ClearRoute,
    InitializeLocalization,
    SetRoutePoints,
)
from autoware_perception_msgs.msg import DetectedObjects, TrafficLightElement
from builtin_interfaces.msg import Time
from decision_counter import summarize_decisions
from fastapi import APIRouter, BackgroundTasks, HTTPException, Request
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovarianceStamped,
    Quaternion,
)
from std_msgs.msg import Header as RosStdHeader

from modules.localization.proto.localization_pb2 import LocalizationEstimate
from violation_analyser.run_analyzer import measure_violations

from .analysis import calculate_min_distance_from_record
from .config import (
    AUTOWARE_LOG_DIR,
    DEFAULT_SENSOR_MODEL,
    DEFAULT_VEHICLE_MODEL,
    RECORD_LOG_DIR,
    REPO_ROOT,
    ROS_NODE_NAME_SERVER,
    VIOLATION_MAP_NAME,
)
from .models import (
    AutowareLaunchRequest,
    LocalizationInitRequest,
    PedestrianBatchRequest,
    PedestrianPublishRequest,
    SetRoutePointsRequest,
    StartLoggingRequest,
    TrafficSignalRequest,
    TrafficSignalsRequest,
    ViolationsCalculateRequest,
)
from .processes import (
    _build_autoware_launch_cmd,
    _resolve_map_path,
    _stop_autoware_process,
    _stop_rosbag_process,
    _stop_sender_process,
)
from .publishers import (
    _build_pedestrian_detected_object,
    _publish_traffic_signal,
    _publish_traffic_signals,
    process_perception_request_async,
    publish_pedestrain,
)
from .state import server_globals

router = APIRouter()


@router.post("/perception")
async def perception_endpoint(request: Request, background_tasks: BackgroundTasks):
    node = server_globals.get("node")
    if not node:
        raise HTTPException(status_code=503, detail="ROS Node not available")

    try:
        request_bytes = await request.body()
        background_tasks.add_task(process_perception_request_async, request_bytes)
        return {
            "status": "ok",
            "message": "Perception data received and scheduled for processing.",
        }
    except Exception as exc:
        if node:
            node.get_logger().error(f"Error in /perception endpoint: {exc}")
        raise HTTPException(
            status_code=500, detail=f"Internal server error: {str(exc)}"
        )


@router.post("/publish_pedestrain")
async def publish_pedestrain_endpoint(req: PedestrianPublishRequest):
    node = server_globals.get("node")
    detection_publisher = server_globals.get("detection_publisher")
    if not node or not detection_publisher:
        raise HTTPException(status_code=503, detail="ROS Node not available")

    if len(req.position) != 3:
        raise HTTPException(status_code=400, detail="position must be [x, y, z]")

    loc = LocalizationEstimate()
    loc.pose.position.x = float(req.position[0])
    loc.pose.position.y = float(req.position[1])
    loc.pose.position.z = float(req.position[2])
    if req.orientation:
        if len(req.orientation) != 4:
            raise HTTPException(
                status_code=400, detail="orientation must be [qx, qy, qz, qw]"
            )
        loc.pose.orientation.qx = float(req.orientation[0])
        loc.pose.orientation.qy = float(req.orientation[1])
        loc.pose.orientation.qz = float(req.orientation[2])
        loc.pose.orientation.qw = float(req.orientation[3])
    else:
        loc.pose.orientation.qx = 0.0
        loc.pose.orientation.qy = 0.0
        loc.pose.orientation.qz = 0.0
        loc.pose.orientation.qw = 1.0
    loc.pose.linear_velocity.x = 0.0
    loc.pose.linear_velocity.y = 0.0
    loc.pose.linear_velocity.z = 0.0
    loc.pose.angular_velocity.x = 0.0
    loc.pose.angular_velocity.y = 0.0
    loc.pose.angular_velocity.z = 0.0
    loc.header.timestamp_sec = time.time()

    publish_pedestrain(
        loc,
        speed_mps=req.speed or 0.0,
        pedestrian_id=req.pedestrian_id,
    )
    return {"status": "ok", "message": "Pedestrian published."}


@router.post("/publish_pedestrians")
async def publish_pedestrians_endpoint(req: PedestrianBatchRequest):
    node = server_globals.get("node")
    detection_publisher = server_globals.get("detection_publisher")
    if not node or not detection_publisher:
        raise HTTPException(status_code=503, detail="ROS Node not available")

    if not req.pedestrians:
        raise HTTPException(status_code=400, detail="pedestrians must be non-empty")

    dets = []
    for ped in req.pedestrians:
        if len(ped.position) != 3:
            raise HTTPException(status_code=400, detail="position must be [x, y, z]")
        if ped.orientation and len(ped.orientation) != 4:
            raise HTTPException(
                status_code=400, detail="orientation must be [qx, qy, qz, qw]"
            )
        orientation = ped.orientation or [0.0, 0.0, 0.0, 1.0]
        dets.append(
            _build_pedestrian_detected_object(
                position=(
                    float(ped.position[0]),
                    float(ped.position[1]),
                    float(ped.position[2]),
                ),
                orientation=(
                    float(orientation[0]),
                    float(orientation[1]),
                    float(orientation[2]),
                    float(orientation[3]),
                ),
                speed_mps=float(ped.speed or 0.0),
                pedestrian_id=ped.pedestrian_id,
            )
        )

    now = time.time()
    out = DetectedObjects()
    out.header = RosStdHeader()
    out.header.stamp = Time(sec=int(now), nanosec=int((now - int(now)) * 1e9))
    out.header.frame_id = "map"
    out.objects = dets

    detection_publisher.publish(out)
    return {"status": "ok", "message": "Pedestrians published."}


@router.post("/traffic_signal")
async def traffic_signal_endpoint(req: TrafficSignalRequest):
    if not server_globals.get("node"):
        raise HTTPException(status_code=503, detail="ROS Node not available")

    state = server_globals["traffic_signal_state"]
    state["map_primitive_id"] = req.map_primitive_id
    if req.color is not None:
        state["color"] = req.color
    if req.shape is not None:
        state["shape"] = req.shape
    if req.status is not None:
        state["status"] = req.status
    if req.confidence is not None:
        state["confidence"] = req.confidence

    _publish_traffic_signal()
    return {"status": "ok", "message": "Traffic signal updated."}


@router.post("/traffic_signals")
async def traffic_signals_endpoint(req: TrafficSignalsRequest):
    if not server_globals.get("node"):
        raise HTTPException(status_code=503, detail="ROS Node not available")

    signals = []
    for item in req.signals:
        signals.append(
            {
                "map_primitive_id": item.map_primitive_id,
                "color": item.color
                if item.color is not None
                else TrafficLightElement.RED,
                "shape": item.shape
                if item.shape is not None
                else TrafficLightElement.CIRCLE,
                "status": item.status
                if item.status is not None
                else TrafficLightElement.SOLID_ON,
                "confidence": item.confidence if item.confidence is not None else 1.0,
            }
        )
    _publish_traffic_signals(signals)
    return {"status": "ok", "message": "Traffic signals updated."}


@router.post("/change_operation_auto_mode")
async def change_operation_auto_mode():
    node = server_globals.get("node")
    client = server_globals.get("change_mode_auto_client")
    if not node or not client:
        raise HTTPException(status_code=503, detail="ROS Node not available")
    node.get_logger().info("Received request to change to AUTONOMUS mode")
    request = ChangeOperationMode.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

    if future.result() is not None:
        response = future.result()
        if response.status.success:
            node.get_logger().info("Successfully change to AUTONOMOUS mode")
            return {"status": "ok", "message": "Mode changed to Auto"}
        node.get_logger().error(
            f"Failed to change to AUTONOMOUS mode: {response.status.message}"
        )
        raise HTTPException(
            status_code=400,
            detail=f"Failed to change to AUTONOMOUS mode: {response.status.message}",
        )

    node.get_logger().error("Service call to change to AUTONOMOUS timed out.")
    raise HTTPException(status_code=504, detail="Service call timed out")


@router.post("/change_operation_stop_mode")
async def change_operation_stop_mode():
    node = server_globals.get("node")
    client = server_globals.get("change_mode_stop_client")
    if not node or not client:
        raise HTTPException(status_code=503, detail="ROS Node not available")
    node.get_logger().info("Received request to change to STOP mode")
    request = ChangeOperationMode.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

    if future.result() is not None:
        response = future.result()
        if response.status.success:
            node.get_logger().info("Successfully change to STOP mode")
            return {"status": "ok", "message": "Mode changed to STOP"}
        node.get_logger().error(
            f"Failed to change to STOP mode: {response.status.message}"
        )
        raise HTTPException(
            status_code=400,
            detail=f"Failed to change to STOP mode: {response.status.message}",
        )

    node.get_logger().error("Service call to change to STOP timed out.")
    raise HTTPException(status_code=504, detail="Service call timed out")


@router.post("/initialize_localization")
async def initialize_localization(init_request: LocalizationInitRequest):
    node = server_globals.get("node")
    client = server_globals.get("init_localization_client")

    if not node or not client:
        raise HTTPException(
            status_code=503, detail="ROS Node or Localization Client not available"
        )
    node.get_logger().info("Received HTTP request -> InitializeLocalization service")

    srv_req = InitializeLocalization.Request()
    pws = PoseWithCovarianceStamped()

    pws.header.stamp = node.get_clock().now().to_msg()
    pws.header.frame_id = "map"

    pos = init_request.pose.position
    ori = init_request.pose.orientation
    pws.pose.pose = Pose(
        position=Point(x=pos[0], y=pos[1], z=pos[2]),
        orientation=Quaternion(x=ori[0], y=ori[1], z=ori[2], w=ori[3]),
    )
    pws.pose.covariance = [0.0] * 36
    srv_req.pose = [pws]

    future = client.call_async(srv_req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

    if future.result() is not None:
        response = future.result()
        if response.status.success:
            node.get_logger().info("InitializeLocalization successful")
            return {"status": "ok", "message": "Localization initialized successfully."}
        node.get_logger().error(
            f"InitializeLocalization failed: {response.status.message}"
        )
        raise HTTPException(
            status_code=400,
            detail=f"Failed to initialize localization: {response.status.message}",
        )

    error_text = (
        f"InitializeLocalization call timed out or failed: {future.exception()}"
    )
    node.get_logger().error(error_text)
    raise HTTPException(
        status_code=504,
        detail="Service call for localization initialization timed out",
    )


@router.post("/set_route_points")
async def set_route_points(route_request: SetRoutePointsRequest):
    node = server_globals.get("node")
    client = server_globals.get("set_route_points_client")
    if not node or not client:
        raise HTTPException(
            status_code=503, detail="ROS Node or Routing Client not available"
        )

    node.get_logger().info("Received HTTP request -> SetRoutePoints service")

    srv_req = SetRoutePoints.Request()

    stamp_data = route_request.header.stamp
    srv_req.header = RosStdHeader(
        stamp=Time(sec=stamp_data[0], nanosec=stamp_data[1]),
        frame_id=route_request.header.frame_id,
    )

    goal_data = route_request.goal
    srv_req.goal = Pose(
        position=Point(
            x=goal_data.position[0],
            y=goal_data.position[1],
            z=goal_data.position[2],
        ),
        orientation=Quaternion(
            x=goal_data.orientation[0],
            y=goal_data.orientation[1],
            z=goal_data.orientation[2],
            w=goal_data.orientation[3],
        ),
    )

    srv_req.waypoints = []
    for wp in route_request.waypoints:
        srv_req.waypoints.append(
            Pose(
                position=Point(x=wp.position[0], y=wp.position[1], z=wp.position[2]),
                orientation=Quaternion(
                    x=wp.orientation[0],
                    y=wp.orientation[1],
                    z=wp.orientation[2],
                    w=wp.orientation[3],
                ),
            )
        )

    future = client.call_async(srv_req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if future.result():
        response = future.result()
        if response.status.success:
            node.get_logger().info("SetRoutePoints successful")
            return {"status": "ok", "message": "Route set successfully."}
        node.get_logger().error(f"SetRoutePoints failed: {response.status.message}")
        raise HTTPException(
            status_code=400,
            detail=f"Failed to set route: {response.status.message}",
        )

    raise HTTPException(status_code=504, detail="Service call to set route timed out")


@router.post("/clear_routes")
async def clear_routes():
    node = server_globals.get("node")
    client = server_globals.get("clear_routes_client")
    if not node or not client:
        raise HTTPException(status_code=503, detail="ROS Node not available")
    node.get_logger().info("Received request to clear routes")
    request = ClearRoute.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

    if future.result() is not None:
        response = future.result()
        if response.status.success:
            node.get_logger().info("Successfully clear routes")
            return {"status": "ok", "message": "clear routes"}
        node.get_logger().error(f"Failed to clear routes: {response.status.message}")
        raise HTTPException(
            status_code=400,
            detail=f"Failed to clear routes: {response.status.message}",
        )

    node.get_logger().error("Service call to clear routes timed out.")
    raise HTTPException(status_code=504, detail="Service call timed out")


@router.post("/sender/start")
async def sender_start():
    node = server_globals.get("node")
    process = server_globals.get("sender_process")
    if process and process.poll() is None:
        raise HTTPException(status_code=409, detail="Sender is already running.")

    sender_path = pathlib.Path(__file__).resolve().parents[1] / "sender.py"
    if not sender_path.exists():
        raise HTTPException(
            status_code=404, detail=f"Sender script not found: {sender_path}"
        )

    try:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "1")
        log_dir = AUTOWARE_LOG_DIR
        log_dir.mkdir(parents=True, exist_ok=True)
        log_file = open(log_dir / "sender.log", "a", encoding="utf-8")
        process = subprocess.Popen(
            ["python3", str(sender_path)],
            cwd=str(sender_path.parent),
            env=env,
            start_new_session=True,
            stdout=log_file,
            stderr=log_file,
        )
        server_globals["sender_process"] = process
        server_globals["sender_log_file"] = log_file
        if node:
            node.get_logger().info(
                f"Started sender. PID: {process.pid}, Path: {sender_path}"
            )
        return {
            "status": "ok",
            "pid": process.pid,
            "log_file": str(log_dir / "sender.log"),
        }
    except Exception as exc:
        try:
            log_file.close()
        except Exception:
            pass
        if node:
            node.get_logger().error(f"Failed to start sender: {exc}")
        raise HTTPException(
            status_code=500, detail=f"Failed to start sender: {str(exc)}"
        )


@router.post("/sender/stop")
async def sender_stop():
    process = server_globals.get("sender_process")
    if not process or process.poll() is not None:
        server_globals["sender_process"] = None
        raise HTTPException(status_code=404, detail="Sender is not running.")

    _stop_sender_process("api stop")
    return {"status": "ok", "message": "Sender stopped."}


@router.post("/sender/restart")
async def sender_restart():
    process = server_globals.get("sender_process")
    if process and process.poll() is None:
        _stop_sender_process("api restart")
    return await sender_start()


@router.get("/sender/status")
async def sender_status():
    process = server_globals.get("sender_process")
    running = process is not None and process.poll() is None
    return {
        "running": running,
        "pid": process.pid if running else None,
    }


@router.post("/autoware/start")
async def autoware_start(request: AutowareLaunchRequest = AutowareLaunchRequest()):
    node = server_globals.get("node")
    process = server_globals.get("autoware_process")
    if process and process.poll() is None:
        raise HTTPException(status_code=409, detail="Autoware is already running.")

    map_path = _resolve_map_path(request.map_path)
    if not map_path.exists():
        raise HTTPException(status_code=404, detail=f"Map path not found: {map_path}")

    vehicle_model = request.vehicle_model or DEFAULT_VEHICLE_MODEL
    sensor_model = request.sensor_model or DEFAULT_SENSOR_MODEL
    command = _build_autoware_launch_cmd(map_path, vehicle_model, sensor_model)

    try:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "1")
        log_dir = AUTOWARE_LOG_DIR
        log_dir.mkdir(parents=True, exist_ok=True)
        log_file = open(log_dir / "autoware_launch.log", "a", encoding="utf-8")
        process = subprocess.Popen(
            command,
            cwd=str(REPO_ROOT),
            env=env,
            start_new_session=True,
            stdout=log_file,
            stderr=log_file,
        )
        server_globals["autoware_process"] = process
        server_globals["autoware_launch_cmd"] = command
        server_globals["autoware_log_file"] = log_file
        if node:
            node.get_logger().info(
                f"Started Autoware launch. PID: {process.pid}, Map: {map_path}"
            )
        return {
            "status": "ok",
            "pid": process.pid,
            "map_path": str(map_path),
            "vehicle_model": vehicle_model,
            "sensor_model": sensor_model,
            "log_file": str(log_dir / "autoware_launch.log"),
        }
    except Exception as exc:
        try:
            log_file.close()
        except Exception:
            pass
        if node:
            node.get_logger().error(f"Failed to start Autoware launch: {exc}")
        raise HTTPException(
            status_code=500, detail=f"Failed to start Autoware launch: {str(exc)}"
        )


@router.post("/autoware/stop")
async def autoware_stop():
    process = server_globals.get("autoware_process")
    if not process or process.poll() is not None:
        server_globals["autoware_process"] = None
        raise HTTPException(status_code=404, detail="Autoware is not running.")

    _stop_autoware_process("api stop")
    return {"status": "ok", "message": "Autoware stopped."}


@router.post("/autoware/restart")
async def autoware_restart(request: AutowareLaunchRequest = AutowareLaunchRequest()):
    process = server_globals.get("autoware_process")
    if process and process.poll() is None:
        _stop_autoware_process("api restart")
    return await autoware_start(request)


@router.get("/autoware/status")
async def autoware_status():
    process = server_globals.get("autoware_process")
    running = process is not None and process.poll() is None
    return {
        "running": running,
        "pid": process.pid if running else None,
        "command": server_globals.get("autoware_launch_cmd"),
    }


@router.post("/logging/start")
async def start_logging(request: StartLoggingRequest):
    node = server_globals.get("node")
    if server_globals.get("rosbag_process"):
        raise HTTPException(
            status_code=409, detail="A recording process is already running. "
        )

    filename = request.filename
    RECORD_LOG_DIR.mkdir(parents=True, exist_ok=True)
    output_dir = RECORD_LOG_DIR / filename
    command = [
        "ros2",
        "bag",
        "record",
        "/tf",
        "/tf_static",
        "/localization/acceleration",
        "/localization/kinematic_state",
        "/perception/object_recognition/objects",
        "/perception/object_recognition/detection/objects",
        "/perception/traffic_light_recognition/traffic_signals",
        "/prediction/objects",
        "/planning/scenario_planning/trajectory",
        "/planning/mission_planning/route",
        "/planning/path_candidate/lane_change_left",
        "/planning/path_candidate/lane_change_right",
        "/planning/planning_factors/behavior_path_planner",
        "/planning/planning_factors/lane_change_left",
        "/planning/planning_factors/lane_change_right",
        "/api/planning/velocity_factors",
        "/control/command/control_cmd",
        "/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/virtual_wall/intersection",
        "/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/virtual_walls",
        "/autoware/state",
        "--output",
        str(output_dir),
    ]
    try:
        process = subprocess.Popen(command)
        server_globals["rosbag_process"] = process
        server_globals["record_output_dir"] = str(output_dir)
        if node:
            node.get_logger().info(
                f"Started rosbag recording, PID: {process.pid}, File: {filename}"
            )
            return {
                "status": "ok",
                "message": f"Recording started with filename {filename}",
            }
    except Exception as exc:
        if node:
            node.get_logger().error(f"Failed to start rosbag record: {exc}")
        raise HTTPException(
            status_code=500, detail=f"Failed to start rosbag record: {str(exc)}"
        )


@router.post("/logging/stop")
async def stop_logging():
    node = server_globals.get("node")
    process = server_globals.get("rosbag_process")
    if not process:
        raise HTTPException(
            status_code=404, detail="No active recording process found to stop"
        )

    try:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        server_globals["rosbag_process"] = None
        if node:
            node.get_logger().info(f"Stopped rosbag recording. PID: {process.pid}")
        return {"status": "ok", "message": "Recording stopped successfully"}
    except subprocess.TimeoutExpired:
        if node:
            node.get_logger().warning(
                f"rosbag record (PID: {process.pid}) did not terminate gracefully. Killing."
            )
        process.kill()
        server_globals["rosbag_process"] = None
        raise HTTPException(
            status_code=500, detail="Recording process did not respond and was killed."
        )
    except Exception as exc:
        if node:
            node.get_logger().error(f"Error while stopping rosbag record: {exc}")
        raise HTTPException(
            status_code=500,
            detail=f"An error occurred while stopping the recording: {str(exc)}",
        )


@router.post("/violations/calculate")
async def calculate_violation(
    req: ViolationsCalculateRequest = ViolationsCalculateRequest(),
):
    node = server_globals.get("node")
    if not node:
        raise HTTPException(status_code=503, detail="ROS Node not available")

    if server_globals.get("rosbag_process"):
        raise HTTPException(
            status_code=409,
            detail="Recording is still in progress. Stop logging before analysis.",
        )

    record_dir = server_globals.get("record_output_dir")
    if not record_dir:
        raise HTTPException(
            status_code=404,
            detail="No rosbag recording has been started yet.",
        )

    record_path = pathlib.Path(record_dir)
    if not record_path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"Recorded bag directory not found: {record_path}",
        )

    node.get_logger().info(
        f"Starting violation analysis for record at {record_path} using map {VIOLATION_MAP_NAME}."
    )
    try:
        route_ids = req.route_lanelet_ids or None
        violations = await asyncio.to_thread(
            measure_violations, VIOLATION_MAP_NAME, record_path, route_ids
        )
    except Exception as exc:
        node.get_logger().error(f"Violation analysis failed: {exc}")
        raise HTTPException(
            status_code=500,
            detail=f"Violation analysis failed: {str(exc)}",
        )

    try:
        min_distance = await asyncio.to_thread(
            calculate_min_distance_from_record, record_path
        )
    except Exception as exc:
        node.get_logger().error(f"Min-distance calculation failed: {exc}")
        min_distance = None

    serialized = [
        {
            "main_type": getattr(v, "main_type", ""),
            "key_label": getattr(v, "key_label", ""),
            "features": getattr(v, "features", {}),
        }
        for v in violations
    ]
    node.get_logger().info(
        f"Violation analysis finished. Detected {len(serialized)} violations."
    )
    return {
        "status": "ok",
        "map": VIOLATION_MAP_NAME,
        "record_directory": str(record_path),
        "violation_count": len(serialized),
        "min_distance": min_distance,
        "violations": serialized,
    }


@router.post("/decisions/calculate")
async def calculate_decisions():
    node = server_globals.get("node")
    if not node:
        raise HTTPException(status_code=503, detail="ROS Node not available")

    if server_globals.get("rosbag_process"):
        raise HTTPException(
            status_code=409,
            detail="Recording is still in progress. Stop logging before analysis.",
        )

    record_dir = server_globals.get("record_output_dir")
    if not record_dir:
        raise HTTPException(
            status_code=404,
            detail="No rosbag recording has been started yet.",
        )

    record_path = pathlib.Path(record_dir)
    if not record_path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"Recorded bag directory not found: {record_path}",
        )

    node.get_logger().info(f"Calculating decision summary for {record_path}.")
    try:
        summary = await asyncio.to_thread(summarize_decisions, record_path)
    except Exception as exc:
        node.get_logger().error(f"Decision summary failed: {exc}")
        raise HTTPException(
            status_code=500,
            detail=f"Decision summary failed: {str(exc)}",
        )

    return {
        "status": "ok",
        "record_directory": str(record_path),
        "decisions": summary.to_dict(),
    }


@router.get("/health")
async def health_check_server():
    return {
        "status": "healthy",
        "server_node": ROS_NODE_NAME_SERVER,
        "timestamp": time.time(),
    }
