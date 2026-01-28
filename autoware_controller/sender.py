import asyncio
import os
import threading
import time
import socket

import httpx  # For asynchronous HTTP requests
import rclpy
from rclpy.node import Node

# ====== Protobuf-based message imports (Apollo) ======
# This sender creates and sends LocalizationEstimate
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.localization.proto.pose_pb2 import Pose as ApolloPose
from modules.common.proto.header_pb2 import Header as ApolloHeader
from modules.common.proto.geometry_pb2 import PointENU, Quaternion as ApolloQ, Point3D

# ROS message imports
# This sender subscribes to Odometry
from nav_msgs.msg import Odometry
# from std_msgs.msg import Header as RosStdHeader # Not needed for sending Apollo Header
# from geometry_msgs.msg import Point, Quaternion, Vector3 # Used implicitly via Odometry

# =============================================================================
# Configuration
# =============================================================================
# URL(s) of the server(s) that will receive the odometry data.
# Configure via environment:
#   RECEIVER_URLS="http://172.17.0.3:5002/perception,http://172.17.0.4:5002/perception"
# or RECEIVER_URL="http://172.17.0.3:5002/perception"
def _get_self_ip() -> str:
    env_ip = os.environ.get("SELF_IP") or os.environ.get("RECEIVER_SELF_IP")
    if env_ip:
        return env_ip.strip()
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except Exception:
        return ""


def _load_receiver_urls():
    urls = os.environ.get("RECEIVER_URLS")
    if urls:
        return [u.strip() for u in urls.split(",") if u.strip()]
    single = os.environ.get("RECEIVER_URL")
    if single:
        return [single.strip()]
    domain_id = os.environ.get("ROS_DOMAIN_ID", "").strip()
    if domain_id == "1":
        targets = ["172.17.0.3", "172.17.0.4"]
    elif domain_id == "2":
        targets = ["172.17.0.2", "172.17.0.4"]
    elif domain_id == "3":
        targets = ["172.17.0.2", "172.17.0.3"]
    else:
        self_ip = _get_self_ip()
        candidates = ["172.17.0.2", "172.17.0.3", "172.17.0.4"]
        targets = [ip for ip in candidates if ip and ip != self_ip]
    return [f"http://{ip}:5002/perception" for ip in targets]


ODOMETRY_RECEIVER_URLS = _load_receiver_urls()
ROS_NODE_NAME_SENDER = "odometry_sender_node"
ROS_ODOMETRY_TOPIC = "/localization/kinematic_state"
HTTP_TIMEOUT_SENDER = 3.0  # Seconds

# =============================================================================
# Global Variables for Shared Resources (Sender-specific)
# =============================================================================
sender_globals = {
    "node": None,
    "http_client": None,
    "async_loop_thread": None,
    "async_event_loop": None,
    "ros_ok_event": threading.Event(),  # To signal ROS spinning to stop
}

if "ROS_DOMAIN_ID" in os.environ:
    os.environ["ROS_DOMAIN_ID"] = os.environ["ROS_DOMAIN_ID"]


# =============================================================================
# Asynchronous HTTP Sending Function
# =============================================================================
async def send_odometry_to_receiver_async(data_bytes: bytes):
    """
    Asynchronously sends odometry data to the configured receiver URL using httpx.
    """
    node = sender_globals.get("node")
    http_client = sender_globals.get("http_client")

    if not node or not http_client:
        print(
            f"[{ROS_NODE_NAME_SENDER}] Error: ROS node or HTTP client not initialized for sending."
        )
        return
    for url in ODOMETRY_RECEIVER_URLS:
        try:
            node.get_logger().debug(
                f"Attempting to send {len(data_bytes)} bytes to {url}"
            )
            response = await http_client.post(
                url, content=data_bytes, timeout=HTTP_TIMEOUT_SENDER
            )
            response.raise_for_status()
            node.get_logger().info(
                f"Odometry successfully sent to {url}. Status: {response.status_code}"
            )
        except httpx.TimeoutException:
            node.get_logger().error(f"Timeout sending odometry to {url}")
        except httpx.RequestError as e:
            node.get_logger().error(f"Error sending odometry to {url}: {e}")
        except Exception as e:
            node.get_logger().error(
                f"Unexpected error sending odometry: {type(e).__name__} - {e}"
            )


# =============================================================================
# ROS 2 Callback
# =============================================================================
def odometry_callback(msg: Odometry):
    """
    ROS subscriber callback for Odometry messages.
    Converts ROS Odometry to Apollo LocalizationEstimate and schedules HTTP POST.
    """
    print(msg)
    node = sender_globals.get("node")
    async_event_loop = sender_globals.get("async_event_loop")

    if not node or not async_event_loop:
        print(
            f"[{ROS_NODE_NAME_SENDER}] Error: ROS node or asyncio event loop not available in odometry_callback."
        )
        return

    node.get_logger().debug("Received Odometry message, preparing to send.")

    stamp_sec = msg.header.stamp.sec
    stamp_nanosec = msg.header.stamp.nanosec
    timestamp = stamp_sec + stamp_nanosec * 1e-9

    apollo_header = ApolloHeader(
        frame_id=msg.header.frame_id or "map",  # Use frame_id from Odometry or default
        timestamp_sec=timestamp,
        module_name="ODOMETRY_SENDER_NODE",  # Identify the source
    )
    position = PointENU(
        x=msg.pose.pose.position.x,
        y=msg.pose.pose.position.y,
        z=msg.pose.pose.position.z,
    )
    orientation = ApolloQ(
        qx=msg.pose.pose.orientation.x,
        qy=msg.pose.pose.orientation.y,
        qz=msg.pose.pose.orientation.z,
        qw=msg.pose.pose.orientation.w,
    )
    linear_vel = Point3D(
        x=msg.twist.twist.linear.x,
        y=msg.twist.twist.linear.y,
        z=msg.twist.twist.linear.z,
    )
    angular_vel = Point3D(
        x=msg.twist.twist.angular.x,
        y=msg.twist.twist.angular.y,
        z=msg.twist.twist.angular.z,
    )

    apollo_pose = ApolloPose(
        position=position,
        orientation=orientation,
        linear_velocity=linear_vel,
        angular_velocity=angular_vel,
    )
    apollo_loc_estimate = LocalizationEstimate(header=apollo_header, pose=apollo_pose)
    data_bytes = apollo_loc_estimate.SerializeToString()

    if async_event_loop.is_running():
        asyncio.run_coroutine_threadsafe(
            send_odometry_to_receiver_async(data_bytes), async_event_loop
        )
    else:
        node.get_logger().warning(
            "Asyncio event loop is not running. Cannot send odometry."
        )


# =============================================================================
# Asyncio Event Loop Thread
# =============================================================================
def run_asyncio_loop(loop: asyncio.AbstractEventLoop):
    """Runs the asyncio event loop."""
    asyncio.set_event_loop(loop)
    try:
        loop.run_forever()
    finally:
        loop.close()


# =============================================================================
# Main Execution (Sender)
# =============================================================================
def main(args=None):
    rclpy.init(args=args)

    sender_globals["node"] = Node(ROS_NODE_NAME_SENDER)
    node = sender_globals["node"]
    node.get_logger().info(f"Node '{ROS_NODE_NAME_SENDER}' initialized.")

    # Setup asyncio event loop in a separate thread
    sender_globals["async_event_loop"] = asyncio.new_event_loop()
    loop_thread = threading.Thread(
        target=run_asyncio_loop, args=(sender_globals["async_event_loop"],), daemon=True
    )
    sender_globals["async_loop_thread"] = loop_thread
    loop_thread.start()
    node.get_logger().info("Asyncio event loop thread started.")

    # Initialize httpx client (can be done after loop is available, or pass loop to it)
    # For simplicity, create it here. It will use the running loop when its methods are called via run_coroutine_threadsafe.
    sender_globals["http_client"] = httpx.AsyncClient()
    node.get_logger().info("HTTPX AsyncClient initialized for sender.")

    # Create ROS subscription
    subscription = node.create_subscription(
        Odometry,
        ROS_ODOMETRY_TOPIC,
        odometry_callback,
        10,  # QoS depth
    )
    node.get_logger().info(f"Subscribed to Odometry topic: {ROS_ODOMETRY_TOPIC}")

    sender_globals["ros_ok_event"].set()

    try:
        node.get_logger().info("Odometry sender node spinning...")
        while rclpy.ok() and sender_globals["ros_ok_event"].is_set():
            rclpy.spin_once(
                node, timeout_sec=0.1
            )  # Keep main thread responsive for signals
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down sender.")
    finally:
        node.get_logger().info("Shutting down odometry sender node...")
        sender_globals[
            "ros_ok_event"
        ].clear()  # Signal asyncio loop to stop if it checks this

        if (
            sender_globals["async_event_loop"]
            and sender_globals["async_event_loop"].is_running()
        ):
            # Schedule client close and stop the loop
            async def close_http_client():
                if sender_globals["http_client"]:
                    await sender_globals["http_client"].aclose()
                    node.get_logger().info("HTTPX AsyncClient closed for sender.")

            future = asyncio.run_coroutine_threadsafe(
                close_http_client(), sender_globals["async_event_loop"]
            )
            try:
                future.result(timeout=5)  # Wait for client to close
            except TimeoutError:
                node.get_logger().warning("Timeout waiting for HTTP client to close.")

            sender_globals["async_event_loop"].call_soon_threadsafe(
                sender_globals["async_event_loop"].stop
            )

        if (
            sender_globals["async_loop_thread"]
            and sender_globals["async_loop_thread"].is_alive()
        ):
            sender_globals["async_loop_thread"].join(timeout=2.0)
            node.get_logger().info("Asyncio event loop thread stopped.")

        if node:
            node.destroy_subscription(subscription)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        node.get_logger().info("Odometry sender node shut down complete.")


if __name__ == "__main__":
    main()
