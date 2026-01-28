import math
from typing import Dict, List, Optional, Tuple

from shapely.geometry import LineString, MultiLineString, Point

from scenario_runner.framework.scenario.pd_agents import PDAgent, PDSection
from scenario_runner.hdmap.MapParser import MapParser

DEFAULT_MAP = "autoware_map/sample-map-planning/lanelet2_map.osm"


class PedestrianManager:
    """
    A simplified modeling of constant speed pedestrians.

    :param PDSection pedestrians: pedestrians to be managed
    """

    pedestrians: PDSection
    __instance = None
    pd_walking_time: List[float]

    def __init__(self, pedestrians: PDSection, map_path: str = DEFAULT_MAP) -> None:
        self.pedestrians = pedestrians
        self.map_path = map_path
        self.pd_walking_time = [0.0 for _ in range(len(pedestrians.pds))]
        self.last_time = 0.0  # Reset per instance to avoid cross-scenario timing issues
        PedestrianManager.__instance = self

    @staticmethod
    def get_instance() -> "PedestrianManager":
        """
        Returns the singleton instance.
        """
        return PedestrianManager.__instance

    def _crosswalk_boundary(self, cw_id: str) -> Optional[List[Tuple[float, float]]]:
        mp = MapParser.get_instance(self.map_path)
        cw = mp.get_crosswalk_by_id(cw_id)
        geom = cw.geometry
        if geom is None or geom.is_empty:
            return None
        if geom.geom_type == "Polygon":
            coords = list(geom.exterior.coords)
        elif geom.geom_type == "LineString":
            coords = list(geom.coords)
        else:
            return None
        if len(coords) < 2:
            return None
        return coords

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Dict[str, float]:
        half = yaw * 0.5
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(half),
            "w": math.cos(half),
        }

    @staticmethod
    def _velocity_from_heading(speed: float, heading: float) -> Dict[str, float]:
        return {
            "x": speed * math.cos(heading),
            "y": speed * math.sin(heading),
            "z": 0.0,
        }

    def calculate_position(
        self, pd: PDAgent, time_spent_walking: float
    ) -> Tuple[Dict, float]:
        """
        Calculate the pedestrian's location and heading at a given time.
        """
        distance = pd.speed * time_spent_walking
        boundary = self._crosswalk_boundary(pd.cw_id)
        if not boundary:
            return ({"x": 0.0, "y": 0.0, "z": 0.0}, 0.0)

        line_list = []
        heading_list = []
        for i in range(len(boundary) - 1):
            p1 = boundary[i]
            p2 = boundary[i + 1]
            line_list.append((p1, p2))
            heading_list.append(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))

        if boundary[0] != boundary[-1]:
            line_list.append((boundary[-1], boundary[0]))
            p1 = boundary[-1]
            p2 = boundary[0]
            heading_list.append(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))

        lines = MultiLineString(line_list)
        boundary_length = lines.length
        if boundary_length <= 0:
            p = boundary[0]
            return ({"x": p[0], "y": p[1], "z": 0.0}, 0.0)

        curr_point = lines.interpolate(distance % boundary_length)
        for i, segment in enumerate(line_list):
            line = LineString(segment)
            if line.distance(curr_point) < 1e-6:
                return (
                    {"x": curr_point.x, "y": curr_point.y, "z": 0.0},
                    heading_list[i],
                )

        for i, (x, y) in enumerate(boundary):
            if Point(x, y).distance(curr_point) < 1e-6:
                heading_idx = min(i, len(heading_list) - 1)
                return (
                    {"x": curr_point.x, "y": curr_point.y, "z": 0.0},
                    heading_list[heading_idx],
                )

        return ({"x": curr_point.x, "y": curr_point.y, "z": 0.0}, 0.0)

    def get_pedestrians(self, curr_time: float) -> List[Dict]:
        """
        Get the location and heading of all pedestrians.
        """
        delta_t = curr_time - self.last_time
        result = []
        for index, pd in enumerate(self.pedestrians.pds):
            if curr_time > pd.start_t:
                self.pd_walking_time[index] += max(delta_t, 0.0)

            position, heading = self.calculate_position(pd, self.pd_walking_time[index])
            p_speed = 0.0 if curr_time < pd.start_t else pd.speed
            orientation = self._yaw_to_quaternion(heading)
            linear_velocity = self._velocity_from_heading(p_speed, heading)
            result.append(
                {
                    "id": index,
                    "speed": p_speed,
                    "position": position,
                    "heading": heading,
                    "orientation": orientation,
                    "linear_velocity": linear_velocity,
                    "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                }
            )
        self.last_time = curr_time
        return result


def main() -> None:
    import argparse
    from random import choice

    parser = argparse.ArgumentParser(
        description="Test pedestrian position calculation on a sample crosswalk."
    )
    parser.add_argument(
        "--map",
        default=DEFAULT_MAP,
        help="Path to the Lanelet2 OSM map.",
    )
    args = parser.parse_args()

    mp = MapParser.get_instance(args.map)
    crosswalks = list(mp.get_crosswalks())
    if not crosswalks:
        print("No crosswalks found.")
        return

    pd = PDAgent.get_one_for_cw(choice(crosswalks), map_path=args.map)
    manager = PedestrianManager(PDSection([pd]), map_path=args.map)
    position, heading = manager.calculate_position(pd, time_spent_walking=1.0)
    speed = pd.speed
    orientation = manager._yaw_to_quaternion(heading)
    linear_velocity = manager._velocity_from_heading(speed, heading)
    print(
        {
            "crosswalk_id": pd.cw_id,
            "position": position,
            "heading": heading,
            "orientation": orientation,
            "linear_velocity": linear_velocity,
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        }
    )


if __name__ == "__main__":
    main()
