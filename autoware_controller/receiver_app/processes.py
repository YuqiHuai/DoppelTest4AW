import pathlib
import signal
import subprocess
from typing import List, Optional

from .config import (
    AUTOWARE_LAUNCH_FILE,
    AUTOWARE_LAUNCH_PACKAGE,
    DEFAULT_MAP_PATH,
    DEFAULT_SENSOR_MODEL,
    DEFAULT_VEHICLE_MODEL,
    REPO_ROOT,
)
from .state import server_globals


def _stop_rosbag_process(reason: str) -> None:
    process = server_globals.get("rosbag_process")
    if not process:
        return
    node = server_globals.get("node")
    try:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        if node:
            node.get_logger().info(
                f"Stopped rosbag recording ({reason}). PID: {process.pid}"
            )
    except subprocess.TimeoutExpired:
        if node:
            node.get_logger().warning(
                f"rosbag record (PID: {process.pid}) did not terminate gracefully during {reason}. Killing."
            )
        process.kill()
    except Exception as exc:
        if node:
            node.get_logger().error(
                f"Error while stopping rosbag record during {reason}: {exc}"
            )
    finally:
        server_globals["rosbag_process"] = None


def _resolve_map_path(map_path: Optional[str]) -> pathlib.Path:
    if not map_path:
        return DEFAULT_MAP_PATH
    path = pathlib.Path(map_path)
    if not path.is_absolute():
        path = (REPO_ROOT / path).resolve()
    return path


def _build_autoware_launch_cmd(
    map_path: pathlib.Path, vehicle_model: str, sensor_model: str
) -> List[str]:
    return [
        "ros2",
        "launch",
        AUTOWARE_LAUNCH_PACKAGE,
        AUTOWARE_LAUNCH_FILE,
        f"map_path:={map_path}",
        f"vehicle_model:={vehicle_model}",
        f"sensor_model:={sensor_model}",
    ]


def _stop_autoware_process(reason: str) -> None:
    process = server_globals.get("autoware_process")
    if not process:
        return
    node = server_globals.get("node")
    try:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=15)
        if node:
            node.get_logger().info(
                f"Stopped Autoware launch ({reason}). PID: {process.pid}"
            )
    except subprocess.TimeoutExpired:
        if node:
            node.get_logger().warning(
                f"Autoware launch (PID: {process.pid}) did not terminate gracefully during {reason}. Killing."
            )
        process.kill()
    except Exception as exc:
        if node:
            node.get_logger().error(
                f"Error while stopping Autoware launch during {reason}: {exc}"
            )
    finally:
        server_globals["autoware_process"] = None
        log_file = server_globals.get("autoware_log_file")
        if log_file:
            try:
                log_file.flush()
                log_file.close()
            except Exception:
                pass
            server_globals["autoware_log_file"] = None


def _stop_sender_process(reason: str) -> None:
    process = server_globals.get("sender_process")
    if not process:
        return
    node = server_globals.get("node")
    try:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        if node:
            node.get_logger().info(f"Stopped sender ({reason}). PID: {process.pid}")
    except subprocess.TimeoutExpired:
        if node:
            node.get_logger().warning(
                f"Sender (PID: {process.pid}) did not terminate gracefully during {reason}. Killing."
            )
        process.kill()
    except Exception as exc:
        if node:
            node.get_logger().error(f"Error while stopping sender during {reason}: {exc}")
    finally:
        server_globals["sender_process"] = None
        log_file = server_globals.get("sender_log_file")
        if log_file:
            try:
                log_file.flush()
                log_file.close()
            except Exception:
                pass
            server_globals["sender_log_file"] = None


def get_autoware_defaults():
    return DEFAULT_VEHICLE_MODEL, DEFAULT_SENSOR_MODEL
