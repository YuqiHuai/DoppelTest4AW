import math
import os
from dataclasses import dataclass
from random import randint, random
from secrets import choice
from typing import Dict, List, Optional, Tuple

import requests
import time
from shapely.geometry import Polygon

from scenario_runner.hdmap.MapParser import MapParser

DEFAULT_MAP = "autoware_map/BorregasAve/lanelet2_map.osm"
INSTANCE_MAX_WAIT_TIME = 15
MAX_ADC_COUNT = 5

# Vehicle dimensions (matching DoppelTest/Apollo)
VEHICLE_LENGTH = 4.933
VEHICLE_WIDTH = 2.11
MIN_VEHICLE_SPACING = 5.0  # Minimum distance between vehicle polygons to avoid collisions


@dataclass
class Orientation:
    x: float
    y: float
    z: float
    w: float

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z, "w": self.w}


@dataclass
class Pose:
    lanelet_id: str
    x: float
    y: float
    z: float
    orientation: Orientation

    def to_dict(self) -> Dict:
        return {
            "lanelet_id": self._lanelet_id_as_number(),
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "orientation": self.orientation.to_dict(),
        }

    def _lanelet_id_as_number(self) -> str:
        return (
            int(self.lanelet_id) if str(self.lanelet_id).isdigit() else self.lanelet_id
        )


def generate_vehicle_polygon(pose: Pose) -> Polygon:
    """
    Generate a Shapely polygon for a vehicle at the given pose.
    
    This is a module-level function that can be used for validation
    and distance checking.
    
    Args:
        pose: Vehicle pose with position and orientation
        
    Returns:
        Shapely Polygon representing the vehicle bounding box
    """
    # Extract yaw from quaternion (z, w components)
    # quaternion: (x, y, z, w) where z = sin(yaw/2), w = cos(yaw/2)
    yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
    
    half_l = VEHICLE_LENGTH / 2.0
    half_w = VEHICLE_WIDTH / 2.0
    sin_h = math.sin(yaw)
    cos_h = math.cos(yaw)
    
    # Four corners of vehicle rectangle
    corners = [
        (half_l * cos_h - half_w * sin_h, half_l * sin_h + half_w * cos_h),
        (-half_l * cos_h - half_w * sin_h, -half_l * sin_h + half_w * cos_h),
        (-half_l * cos_h + half_w * sin_h, -half_l * sin_h - half_w * cos_h),
        (half_l * cos_h + half_w * sin_h, half_l * sin_h - half_w * cos_h),
    ]
    
    # Translate to vehicle position
    polygon_points = [(pose.x + dx, pose.y + dy) for dx, dy in corners]
    return Polygon(polygon_points)


@dataclass
class ADAgent:
    """
    Genetic representation of a single Autoware ADS instance.

    :param List[str] route: list of lanelets expected to travel on
    :param Pose start_pose: starting pose on the first lanelet
    :param Pose goal_pose: goal pose on the last lanelet
    :param float start_t: when should the instance start
    """

    route: List[str]
    start_pose: Pose
    goal_pose: Pose
    start_t: float

    def to_dict(self) -> Dict:
        return {
            "route": [int(r) if str(r).isdigit() else r for r in self.route],
            "start_pose": self.start_pose.to_dict(),
            "goal_pose": self.goal_pose.to_dict(),
        }

    @staticmethod
    def _yaw_to_orientation(yaw: float) -> Orientation:
        return Orientation(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    @staticmethod
    def _pose_from_lane_s(mp: MapParser, lane_id: str, s: float) -> Pose:
        point, yaw = mp.get_coordinate_and_heading(lane_id, s)
        orientation = ADAgent._yaw_to_orientation(yaw)
        return Pose(
            lanelet_id=lane_id,
            x=point.x,
            y=point.y,
            z=point.z,
            orientation=orientation,
        )

    @staticmethod
    def get_one(
        map_path: str = DEFAULT_MAP, must_not_start_from_junction: bool = True
    ) -> "ADAgent":
        """
        Randomly generates an Autoware ADS instance representation.
        """
        mp = MapParser.get_instance(map_path)
        if must_not_start_from_junction:
            allowed_start = list(mp.get_lanes_not_in_junction())
        else:
            allowed_start = list(mp.get_lanes())

        start_lane = ""
        route: Optional[List[str]] = None
        while True:
            start_lane = choice(allowed_start)
            allowed_routing = mp.get_path_from(start_lane)
            if allowed_routing:
                route = choice(allowed_routing)
                break

        start_length = mp.get_lane_length(start_lane)
        dest_length = mp.get_lane_length(route[-1])

        if start_length > 5:
            start_s = round(random() * (start_length - 5), 1)
        else:
            start_s = 0.0

        dest_s = round(dest_length / 2, 1)
        start_pose = ADAgent._pose_from_lane_s(mp, start_lane, start_s)
        goal_pose = ADAgent._pose_from_lane_s(mp, route[-1], dest_s)

        return ADAgent(
            route=route,
            start_pose=start_pose,
            goal_pose=goal_pose,
            start_t=randint(0, INSTANCE_MAX_WAIT_TIME),
        )

    @staticmethod
    def get_one_for_route(route: List[str], map_path: str = DEFAULT_MAP) -> "ADAgent":
        """
        Get an ADS instance representation with the specified route.
        """
        start_lane = route[0]
        mp = MapParser.get_instance(map_path)
        start_length = mp.get_lane_length(start_lane)
        dest_length = mp.get_lane_length(route[-1])

        if start_length > 5:
            start_s = round(random() * (start_length - 5), 1)
        else:
            start_s = 0.0

        dest_s = round(dest_length / 2, 1)
        start_pose = ADAgent._pose_from_lane_s(mp, start_lane, start_s)
        goal_pose = ADAgent._pose_from_lane_s(mp, route[-1], dest_s)

        return ADAgent(
            route=route,
            start_pose=start_pose,
            goal_pose=goal_pose,
            start_t=randint(0, INSTANCE_MAX_WAIT_TIME),
        )


@dataclass
class ADSection:
    """
    Genetic representation of the ADS instance section.

    :param List[ADAgent] adcs: list of ADS instance representations
    """

    adcs: List[ADAgent]

    def adjust_time(self):
        self.adcs.sort(key=lambda x: x.start_t)
        start_times = [x.start_t for x in self.adcs]
        if not start_times:
            return
        delta = round(start_times[0] - 2.0, 1)
        for i in range(len(start_times)):
            start_times[i] = round(start_times[i] - delta, 1)
            self.adcs[i].start_t = start_times[i]

    def add_agent(self, adc: ADAgent, min_distance: float = MIN_VEHICLE_SPACING) -> bool:
        """
        Add an agent to the section if it's not too close to existing agents.
        
        Uses polygon-to-polygon distance checking to ensure vehicles don't overlap
        or start too close together.
        
        Args:
            adc: The agent to add
            min_distance: Minimum distance between vehicle polygons (default: MIN_VEHICLE_SPACING)
            
        Returns:
            True if successfully added, False if too close to existing agents
        """
        try:
            new_poly = generate_vehicle_polygon(adc.start_pose)
            for ad in self.adcs:
                existing_poly = generate_vehicle_polygon(ad.start_pose)
                distance = new_poly.distance(existing_poly)
                if distance < min_distance:
                    return False
            self.adcs.append(adc)
            return True
        except Exception as e:
            # If polygon generation fails, fall back to simple distance check
            for ad in self.adcs:
                dx = ad.start_pose.x - adc.start_pose.x
                dy = ad.start_pose.y - adc.start_pose.y
                if (dx * dx + dy * dy) ** 0.5 < min_distance:
                    return False
            self.adcs.append(adc)
            return True

    def has_conflict(self, adc: ADAgent, map_path: str = DEFAULT_MAP) -> bool:
        mp = MapParser.get_instance(map_path)
        for ad in self.adcs:
            if ad is adc:
                continue
            if mp.is_conflict_lanes(adc.route, ad.route):
                return True
        return False

    @staticmethod
    def get_one(map_path: str = DEFAULT_MAP) -> "ADSection":
        num = MAX_ADC_COUNT
        result = ADSection([])
        restrict_junction_start = True
        trial_count = 0
        max_trials = 50 * num  # Increased limit to allow more attempts
        while len(result.adcs) < num and trial_count < max_trials:
            new_agent = ADAgent.get_one(map_path, restrict_junction_start)
            if result.add_agent(new_agent):
                # Successfully added, reset trial count for next agent
                trial_count = 0
            else:
                # Failed to add (too close), increment trial count
                trial_count += 1
                if trial_count > 3 * num:
                    restrict_junction_start = False
        
        # If we couldn't add enough agents, adjust time anyway
        if len(result.adcs) > 0:
            result.adjust_time()
        return result


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Generate a sample Autoware ADAgent.")
    parser.add_argument(
        "--map",
        default=DEFAULT_MAP,
        help="Path to the Lanelet2 OSM map.",
    )
    parser.add_argument(
        "--allow-junction-start",
        action="store_true",
        help="Allow starting inside intersection lanelets.",
    )
    parser.add_argument(
        "--vehicle-url",
        default=os.environ.get("AUTOWARE_RECEIVER_URL", "http://172.17.0.2:5002"),
        help=(
            "HTTP base URL for one receiver (default: AUTOWARE_RECEIVER_URL or "
            "Docker bridge example http://172.17.0.2:5002)."
        ),
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=3.0,
        help="Seconds to wait between operations.",
    )
    args = parser.parse_args()

    def _pose_to_payload(pose: Pose) -> Dict[str, List[float]]:
        return {
            "position": [pose.x, pose.y, pose.z],
            "orientation": [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
        }

    def _post(path: str, json_body: Optional[dict] = None) -> dict:
        url = f"{args.vehicle_url.rstrip('/')}{path}"
        resp = requests.post(url, json=json_body, timeout=5.0)
        resp.raise_for_status()
        return resp.json()

    section = ADSection.get_one(map_path=args.map)
    print(f"Generated section with {len(section.adcs)} agents.")
    success_count = 0
    last_error: Optional[Exception] = None

    for idx, agent in enumerate(section.adcs, start=1):
        print(f"Agent {idx} route:", agent.route)
        print(f"Agent {idx} start_pose:", agent.start_pose.to_dict())
        print(f"Agent {idx} goal_pose:", agent.goal_pose.to_dict())
        start_payload = _pose_to_payload(agent.start_pose)
        goal_payload = _pose_to_payload(agent.goal_pose)
        now = time.time()
        header = {
            "stamp": [int(now), int((now - int(now)) * 1e9)],
            "frame_id": "map",
        }
        route_payload = {
            "header": header,
            "goal": goal_payload,
            "waypoints": [start_payload, goal_payload],
        }

        try:
            _post("/change_operation_stop_mode")
            time.sleep(args.delay)
            _post("/clear_routes")
            time.sleep(args.delay)
            init_resp = _post("/initialize_localization", {"pose": start_payload})
            print(f"initialize_localization response: {init_resp}")
            time.sleep(args.delay)
            route_resp = _post("/set_route_points", route_payload)
            print(f"set_route_points response: {route_resp}")
            success_count += 1
        except Exception as exc:
            last_error = exc
            print(f"set_route_points for agent {idx} failed: {exc}")
        time.sleep(args.delay)

    print(
        f"set_route_points summary: {success_count}/{len(section.adcs)} succeeded "
        f"(last_error={last_error})"
    )


if __name__ == "__main__":
    main()
