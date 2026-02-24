from geometry_msgs.msg import Point
from tools.utils import distance


class OracleHelper:

    def __init__(self):
        self.ego_pose_pts = set()
        self.routing_plan = None
        self.route_lanelet_ids = set()
        self._explicit_route_ids = False  # True if set via API, should not be overwritten

    def add_ego_pose_pt(self, p):
        self.ego_pose_pts.add(MyPoint(p))

    def set_routing_plan(self, msg):
        self.routing_plan = (msg.start_pose, msg.goal_pose)
        # Only extract route IDs from rosbag if no explicit IDs were provided via API
        if not self._explicit_route_ids:
            self.route_lanelet_ids = self._extract_lanelet_ids(msg)

    def set_route_lanelet_ids(self, lanelet_ids):
        self.route_lanelet_ids = set(int(v) for v in lanelet_ids or [])
        self._explicit_route_ids = bool(self.route_lanelet_ids)

    def get_route_lanelet_ids(self):
        return set(self.route_lanelet_ids)

    @staticmethod
    def _extract_lanelet_ids(msg):
        ids = set()
        segments = getattr(msg, "segments", None)
        if segments:
            for seg in segments:
                # Autoware LaneletRouteSegment commonly stores lane IDs in preferred_primitive.id.
                preferred = getattr(seg, "preferred_primitive", None)
                if preferred is not None:
                    pid = getattr(preferred, "id", None)
                    if isinstance(pid, int):
                        ids.add(pid)
                primitives = getattr(seg, "primitives", None)
                if primitives:
                    for prim in primitives:
                        pid = getattr(prim, "id", None)
                        if isinstance(pid, int):
                            ids.add(pid)
                for attr in ("preferred_lane_id", "lane_id", "lanelet_id"):
                    val = getattr(seg, attr, None)
                    if isinstance(val, int):
                        ids.add(val)
                for attr in ("lanelet_ids", "lane_ids", "preferred_lane_ids"):
                    vals = getattr(seg, attr, None)
                    if vals:
                        ids.update(int(v) for v in vals)
                alternatives = getattr(seg, "alternatives", None)
                if alternatives:
                    for alt in alternatives:
                        for attr in ("preferred_lane_id", "lane_id", "lanelet_id"):
                            val = getattr(alt, attr, None)
                            if isinstance(val, int):
                                ids.add(val)
        return ids

    def has_routing_plan(self):
        return self.routing_plan is not None or bool(self.route_lanelet_ids)

    def has_enough_ego_poses(self):
        if self.routing_plan is None:
            return len(self.ego_pose_pts) >= 3 or bool(self.route_lanelet_ids)
        if len(self.ego_pose_pts) < 3:
            if MyPoint(self.routing_plan[0]) in self.ego_pose_pts \
                    and MyPoint(self.routing_plan[1]) in self.ego_pose_pts \
                    and distance(self.routing_plan[0], self.routing_plan[1]) < 5:
                return True
            else:
                return False
        return True


class MyPoint(Point):
    def __init__(self, point):
        super().__init__(x=point.x, y=point.y, z=point.z)

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __eq__(self, other):
        o = Point(x=other.x, y=other.y, z=other.z)
        return super().__eq__(o)

    def __repr__(self):
        return super().__repr__()
