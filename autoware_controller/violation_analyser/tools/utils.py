import glob
import math
import os
import time
try:
    from autoware_perception_msgs.msg import PredictedObject
except Exception:
    class PredictedObject:  # type: ignore
        pass
from geometry_msgs.msg import Point, Quaternion
from config import (AUTOWARE_VEHICLE_LENGTH, AUTOWARE_VEHICLE_back_edge_to_center,
                    AUTOWARE_VEHICLE_WHEEL_BASE, AUTOWARE_VEHICLE_WIDTH)
from shapely.geometry import Polygon, LineString, Point as ShapelyPoint
from std_msgs.msg import Header



def quaternion_2_heading(orientation: Quaternion) -> float:
    """
    Convert quaternion to heading

    Parameters:
        orientation: Quaternion
            quaternion of the car

    Returns:
        The heading value of the car

    """

    def normalize_angle(angle):
        a = math.fmod(angle + math.pi, 2.0 * math.pi)
        if a < 0.0:
            a += (2.0 * math.pi)
        return a - math.pi

    yaw = math.atan2(2.0 * (orientation.w * orientation.z - orientation.x * orientation.y),
                     2.0 * (orientation.w * orientation.w + orientation.y * orientation.y) - 1.0)
    return normalize_angle(yaw)


def generate_polygon(position: Point, theta: float, length: float, width: float):
    """
    Generate polygon for a perception obstacle

    Parameters:
        position: Point
            position vector of the obstacle
        theta: float
            heading of the obstacle
        length: float
            length of the obstacle
        width: float
            width of the obstacle

    Returns:
        points: List[Point]
            polygon points of the obstacle
    """
    points = []
    half_l = length / 2.0
    half_w = width / 2.0
    sin_h = math.sin(theta)
    cos_h = math.cos(theta)
    vectors = [(half_l * cos_h - half_w * sin_h,
                half_l * sin_h + half_w * cos_h),
               (-half_l * cos_h - half_w * sin_h,
                - half_l * sin_h + half_w * cos_h),
               (-half_l * cos_h + half_w * sin_h,
                - half_l * sin_h - half_w * cos_h),
               (half_l * cos_h + half_w * sin_h,
                half_l * sin_h - half_w * cos_h)]
    for x, y in vectors:
        p = Point()
        p.x = position.x + x
        p.y = position.y + y
        p.z = position.z
        points.append(p)
    return points


def generate_adc_polygon(position: Point, theta: float):
    """
    Generate polygon for the ADC

    Parameters:
        position: Point
            localization pose of ADC
        theta: float
            heading of ADC

    Returns:
        points: List[Point]
            polygon points of the ADC
    """
    points = []
    half_w = AUTOWARE_VEHICLE_WIDTH / 2.0
    front_l = AUTOWARE_VEHICLE_LENGTH - AUTOWARE_VEHICLE_back_edge_to_center
    back_l = -1 * AUTOWARE_VEHICLE_back_edge_to_center
    sin_h = math.sin(theta)
    cos_h = math.cos(theta)
    vectors = [(front_l * cos_h - half_w * sin_h,
                front_l * sin_h + half_w * cos_h),
               (back_l * cos_h - half_w * sin_h,
                back_l * sin_h + half_w * cos_h),
               (back_l * cos_h + half_w * sin_h,
                back_l * sin_h - half_w * cos_h),
               (front_l * cos_h + half_w * sin_h,
                front_l * sin_h - half_w * cos_h)]
    for x, y in vectors:
        p = Point()
        p.x = position.x + x
        p.y = position.y + y
        p.z = position.z
        points.append(p)
    return points


def generate_adc_rear_vertices(position: Point, theta: float):
    """
    Generate rear for the ADC

    Parameters:
        position: Point
            localization pose of ADC
        theta: float
            heading of ADC

    Returns:
        points: List[Point]
            polygon points of the ADC
    """
    points = []
    half_w = AUTOWARE_VEHICLE_WIDTH / 2.0
    back_l = -1 * AUTOWARE_VEHICLE_back_edge_to_center
    sin_h = math.sin(theta)
    cos_h = math.cos(theta)
    vectors = [(back_l * cos_h - half_w * sin_h,
                back_l * sin_h + half_w * cos_h),
               (back_l * cos_h + half_w * sin_h,
                back_l * sin_h - half_w * cos_h)]

    for x, y in vectors:
        p = Point()
        p.x = position.x + x
        p.y = position.y + y
        p.z = position.z
        points.append(p)
    return points


def obstacle_to_polygon(obs: PredictedObject) -> Polygon:
    """
    Generate polygon for the Obstacle Object

    Parameters:
        obs: PredictedObject
            predicted object of the obstacle

    Returns:
        points: Polygon
            polygon of the obstacle
    """
    if obs.shape.type == 1:
        radius = max(obs.shape.dimensions.x, obs.shape.dimensions.y) / 2.0
        center = obs.kinematics.initial_pose_with_covariance.pose.position
        return ShapelyPoint(center.x, center.y).buffer(radius, resolution=16)
    obs_heading = quaternion_2_heading(obs.kinematics.initial_pose_with_covariance.pose.orientation)
    points = []
    half_w = obs.shape.dimensions.y / 2.0
    front_l = obs.shape.dimensions.x / 2.0
    # back_l of obstacles is half of the length
    back_l = -1 * obs.shape.dimensions.x / 2.0
    sin_h = math.sin(obs_heading)
    cos_h = math.cos(obs_heading)
    vectors = [(front_l * cos_h - half_w * sin_h,
                front_l * sin_h + half_w * cos_h),
               (back_l * cos_h - half_w * sin_h,
                back_l * sin_h + half_w * cos_h),
               (back_l * cos_h + half_w * sin_h,
                back_l * sin_h - half_w * cos_h),
               (front_l * cos_h + half_w * sin_h,
                front_l * sin_h - half_w * cos_h)]
    for x, y in vectors:
        p = Point()
        p.x = obs.kinematics.initial_pose_with_covariance.pose.position.x + x
        p.y = obs.kinematics.initial_pose_with_covariance.pose.position.y + y
        p.z = obs.kinematics.initial_pose_with_covariance.pose.position.z
        points.append(p)

    return Polygon([[x.x, x.y] for x in points])

def to_Point(data):
    return Point(
        x=0.0 if math.isnan(data.x) else data.x,
        y=0.0 if math.isnan(data.y) else data.y,
        z=0.0 if math.isnan(data.z) else data.z
    )


def calculate_velocity(linear_velocity: Point):
    x, y, z = linear_velocity.x, linear_velocity.y, linear_velocity.z
    return round(math.sqrt(x ** 2 + y ** 2), 2)


# TODO
def construct_lane_polygon(lane_msg):
    '''
    Construct the lane polygon based on their boundaries
    '''
    left_points = get_lane_boundary_points(lane_msg.left_boundary)
    right_points = get_lane_boundary_points(lane_msg.right_boundary)
    right_points.reverse()
    all_points = left_points + right_points
    return Polygon(all_points)


def get_lane_boundary_points(boundary):
    '''
    Given a lane boundary (left/right), return a list of x, y
    coordinates of all points in the boundary
    '''
    return [(pt.x, pt.y) for pt in boundary]


def construct_lane_boundary_linestring(lane):
    """
    Description: Construct two linestrings for the lane's left and right boundary
    Input: A lane message.
    Output: A list containing the linestrings representing the left and right boundary of the lane
    """
    left_boundary_points = get_lane_boundary_points(lane.leftBound)
    right_boundary_points = get_lane_boundary_points(lane.rightBound)
    return LineString(left_boundary_points), LineString(right_boundary_points)


# TODO
def find_all_files_by_wildcard(base_dir, file_name, recursive=False):
    # NOTE: combine recursive and **/ to matches all files in the current directory and in all subdirectories
    return glob.glob(join_path(base_dir, file_name), recursive=recursive)


def join_path(*args, **kwargs):
    return os.path.join(*args, **kwargs)


def get_current_timestamp():
    return round(time.time())


def get_real_time_from_msg(header: Header):
    return header.stamp.sec * 1000000000 + header.stamp.nanosec


def distance(p1: Point, p2: Point):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


# --------------------------------------------------------------- stop lines --

class StopLineCrossings:
    """Where a vehicle is relative to a set of stop lines, and when it passes one.

    Shared by the traffic-signal and stop-sign oracles so that "beyond the stop
    line" has ONE definition. Two choices in it are load-bearing, and both were
    made after watching correct behaviour be reported as a violation:

    - The reference point is the FRONT AXLE, not the vehicle footprint.
      Autoware stops at a stop line with `stop_margin: 0.0`, i.e. its target is
      the front bumper ON the line, held there. Anything anchored at the bumper
      therefore triggers on every correct stop. The front axle is a wheel_base
      behind the bumper and stays clear -- measured, by 0.46-0.81 m.

    - What is reported is a CROSSING, not the state of being beyond the line.
      A vehicle already past the line when a signal turns red, or when the
      recording starts, has not crossed anything.
    """

    # How far past either end of a stop line the crossing may lie and still
    # count. A stop line spans its own approach, so a vehicle outside this
    # window is on a different road.
    SPAN_TOLERANCE_M = 1.0

    def __init__(self, stop_line_geometries):
        self.frames = {}
        for key, geom in stop_line_geometries.items():
            frame = stop_line_frame(geom)
            if frame is not None:
                self.frames[key] = frame
        self._orientation = {}
        self._last_offset = {}

    def update(self, localization):
        """Consume one odometry sample.

        Returns (offsets, crossed): the signed distance from each stop line to
        the front axle -- positive beyond it, None when the axle is outside the
        line's extent -- and the ids whose line the axle crossed on this sample.
        """
        pose = localization.pose.pose
        heading = quaternion_2_heading(pose.orientation)
        heading_x, heading_y = math.cos(heading), math.sin(heading)
        axle_x = pose.position.x + AUTOWARE_VEHICLE_WHEEL_BASE * heading_x
        axle_y = pose.position.y + AUTOWARE_VEHICLE_WHEEL_BASE * heading_y

        offsets = {}
        crossed = []
        for key, frame in self.frames.items():
            offset = self._offset(key, frame, axle_x, axle_y, heading_x, heading_y)
            previous = self._last_offset.get(key)
            self._last_offset[key] = offset
            offsets[key] = offset
            # Both samples must lie within the line's extent: without a
            # `before` there is nothing to have crossed.
            if offset is not None and previous is not None and previous <= 0.0 < offset:
                crossed.append(key)
        return offsets, crossed

    def _offset(self, key, frame, axle_x, axle_y, heading_x, heading_y):
        (x0, y0), (tx, ty), (nx, ny), span = frame
        rx, ry = axle_x - x0, axle_y - y0
        along = rx * tx + ry * ty
        if along < -self.SPAN_TOLERANCE_M or along > span + self.SPAN_TOLERANCE_M:
            return None
        # A stop line's normal has no inherent direction -- which side is
        # "beyond" depends on which way the vehicle drives -- so fix it once,
        # from the heading of the first approach. Re-deriving it every sample
        # would flip the sign, and so fake a crossing, when a vehicle turns.
        orientation = self._orientation.get(key)
        if orientation is None:
            orientation = 1.0 if (nx * heading_x + ny * heading_y) > 0 else -1.0
            self._orientation[key] = orientation
        return (rx * nx + ry * ny) * orientation


def stop_line_frame(geom):
    """Origin, tangent, normal and span of a stop line, or None if degenerate."""
    coords = []
    geom_type = getattr(geom, "geom_type", "")
    if geom_type == "LineString":
        coords = list(geom.coords)
    elif geom_type == "MultiLineString":
        for line in getattr(geom, "geoms", []):
            coords.extend(list(line.coords))
    if len(coords) < 2:
        return None
    x0, y0 = float(coords[0][0]), float(coords[0][1])
    x1, y1 = float(coords[-1][0]), float(coords[-1][1])
    dx, dy = x1 - x0, y1 - y0
    span = math.hypot(dx, dy)
    if span < 1e-6:
        return None
    return (x0, y0), (dx / span, dy / span), (-dy / span, dx / span), span
