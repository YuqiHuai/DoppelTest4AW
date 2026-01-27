import threading
from autoware_perception_msgs.msg import TrafficLightElement

server_globals = {
    "node": None,
    "detection_publisher": None,
    "change_mode_auto_client": None,
    "change_mode_stop_client": None,
    "init_localization_client": None,
    "set_route_points_client": None,
    "clear_routes_client": None,
    "rosbag_process": None,
    "ros_thread": None,
    "ros_ok_event": threading.Event(),
    "record_output_dir": None,
    "traffic_signal_publisher": None,
    "autoware_process": None,
    "autoware_launch_cmd": None,
    "autoware_log_file": None,
    "sender_process": None,
    "sender_log_file": None,
    "traffic_signal_state": {
        "map_primitive_id": 301,
        "color": TrafficLightElement.RED,
        "shape": TrafficLightElement.CIRCLE,
        "status": TrafficLightElement.SOLID_ON,
        "confidence": 1.0,
    },
}
