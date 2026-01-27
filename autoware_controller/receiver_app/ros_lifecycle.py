import threading

import rclpy
from autoware_adapi_v1_msgs.srv import (
    ChangeOperationMode,
    ClearRoute,
    InitializeLocalization,
    SetRoutePoints,
)
from autoware_perception_msgs.msg import DetectedObjects, TrafficLightGroupArray
from fastapi import FastAPI
from rclpy.node import Node

from .config import (
    ROS_DETECTION_TOPIC,
    ROS_NODE_NAME_SERVER,
    ROS_TRAFFIC_SIGNAL_TOPIC,
)
from .logging_setup import ros_info_logger
from .state import server_globals


def spin_ros_node_server() -> None:
    node = server_globals.get("node")
    ros_ok_event = server_globals.get("ros_ok_event")

    if not node:
        print(
            f"[{ROS_NODE_NAME_SERVER}] Error: ROS node not initialized in spin_ros_node_server."
        )
        return

    node.get_logger().info("ROS spinning thread for server started.")
    while rclpy.ok() and ros_ok_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info("ROS spinning thread for server stopped.")


def register_lifecycle_handlers(app: FastAPI) -> None:
    @app.on_event("startup")
    async def startup_event_server():
        if not rclpy.ok():
            rclpy.init(args=None)

        server_globals["node"] = Node(ROS_NODE_NAME_SERVER)
        node = server_globals["node"]
        node.get_logger().info(
            f"Node '{ROS_NODE_NAME_SERVER}' initialized successfully."
        )

        class _InfoToFileLogger:
            def __init__(self, ros_logger, py_logger):
                self._ros = ros_logger
                self._py = py_logger

            def info(self, msg, *a, **kw):
                self._ros.info(msg)
                self._py.info(str(msg))

            def debug(self, msg, *a, **kw):
                self._ros.debug(msg)

            def warn(self, msg, *a, **kw):
                self._ros.warn(msg)

            def warning(self, msg, *a, **kw):
                self._ros.warning(msg)

            def error(self, msg, *a, **kw):
                self._ros.error(msg)

            def fatal(self, msg, *a, **kw):
                self._ros.fatal(msg)

        node._orig_logger = node.get_logger()
        node.get_logger = lambda _l=_InfoToFileLogger(
            node._orig_logger, ros_info_logger
        ): _l

        server_globals["detection_publisher"] = node.create_publisher(
            DetectedObjects, ROS_DETECTION_TOPIC, 10
        )
        node.get_logger().info(
            f"Detection publisher created for topic {ROS_DETECTION_TOPIC}."
        )

        server_globals["traffic_signal_publisher"] = node.create_publisher(
            TrafficLightGroupArray, ROS_TRAFFIC_SIGNAL_TOPIC, 10
        )
        node.get_logger().info(
            f"Traffic signal publisher created for topic {ROS_TRAFFIC_SIGNAL_TOPIC}."
        )

        auto_mode_service = "/api/operation_mode/change_to_autonomous"
        server_globals["change_mode_auto_client"] = node.create_client(
            ChangeOperationMode, auto_mode_service
        )
        print("auto mode service started")
        node.get_logger().info(f"Client created for server {auto_mode_service}")

        stop_mode_service = "/api/operation_mode/change_to_stop"
        server_globals["change_mode_stop_client"] = node.create_client(
            ChangeOperationMode, stop_mode_service
        )
        print("stop mode service started")
        node.get_logger().info(f"Client created for server {stop_mode_service}")

        init_loc_service = "/api/localization/initialize"
        server_globals["init_localization_client"] = node.create_client(
            InitializeLocalization, init_loc_service
        )
        node.get_logger().info(f"Client created for service {init_loc_service}")

        set_route_points_service = "/api/routing/set_route_points"
        server_globals["set_route_points_client"] = node.create_client(
            SetRoutePoints, set_route_points_service
        )
        node.get_logger().info(f"Client created for service {set_route_points_service}")

        clear_routes_service = "/api/routing/clear_route"
        server_globals["clear_routes_client"] = node.create_client(
            ClearRoute, clear_routes_service
        )
        node.get_logger().info(f"Client created for service {clear_routes_service}")

        server_globals["ros_ok_event"].set()
        ros_thread = threading.Thread(target=spin_ros_node_server, daemon=True)
        server_globals["ros_thread"] = ros_thread
        ros_thread.start()
        node.get_logger().info("ROS spinning thread for server initiated.")

    @app.on_event("shutdown")
    async def shutdown_event_server():
        node = server_globals.get("node")
        ros_thread = server_globals.get("ros_thread")
        ros_ok_event = server_globals.get("ros_ok_event")

        if node:
            node.get_logger().info(f"[{ROS_NODE_NAME_SERVER}] Shutdown initiated...")

        if ros_ok_event:
            ros_ok_event.clear()
        if ros_thread and ros_thread.is_alive():
            ros_thread.join(timeout=2.0)
            if node:
                node.get_logger().info("ROS spinning thread for server stopped.")

        from .processes import (
            _stop_autoware_process,
            _stop_rosbag_process,
            _stop_sender_process,
        )

        _stop_rosbag_process("shutdown")
        _stop_autoware_process("shutdown")
        _stop_sender_process("shutdown")

        if rclpy.ok():
            if node:
                perception_publisher = server_globals.get("perception_publisher")
                if perception_publisher:
                    node.destroy_publisher(perception_publisher)
                traffic_signal_publisher = server_globals.get(
                    "traffic_signal_publisher"
                )
                if traffic_signal_publisher:
                    node.destroy_publisher(traffic_signal_publisher)
                node.destroy_node()
            rclpy.shutdown()
            if node:
                node.get_logger().info("RCLPY shut down successfully for server.")

        print(f"[{ROS_NODE_NAME_SERVER}] Application shutdown complete.")
