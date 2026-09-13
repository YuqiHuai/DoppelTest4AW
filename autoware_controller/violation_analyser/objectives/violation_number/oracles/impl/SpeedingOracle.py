from datetime import datetime
from itertools import groupby
from typing import List
from lanelet2.core import BasicPoint2d, BasicPoint3d
from lanelet2.geometry import inside
from objectives.violation_number.oracles.OracleInterface import OracleInterface
from objectives.violation_number.oracles.Violation import Violation
from tools.hdmap.VectorMapParser import VectorMapParser
from tools.autoware_tools.calculate_velocity import calculate_velocity


class SpeedingOracle(OracleInterface):
    """
    Speeding Oracle is responsible for checking if the ego vehicle violates speed limit at any point
    Its features include:
        * x:            float
        * y:            float
        * heading:      float
        * speed:        float
        * speed_limit:  float
        * duration:     float
    """

    #: Fraction over a lanelet's posted limit before the ego is speeding.
    #:
    #: 10%, not the 2.5% this carried through run6-run8. The tolerance is
    #: proportional and the error it has to absorb is not: 2.5% of a 50 km/h
    #: lane is 1.25 km/h, but 2.5% of a 5 km/h lane is 0.125 km/h -- 3.5 cm/s,
    #: below what the controller holds. Measured on those runs, the ego settles
    #: 2.6-3.0% above whatever limit is constraining it (limit 8 in run8: all
    #: 470 findings at exactly 8.21 km/h), so 2.5% reported that steady-state
    #: offset as 1941 violations on the only lanelets whose limit sits below
    #: the ego's cruising speed -- 5, 8 and 10 km/h, never 30 or above.
    #:
    #: At 10% those collapse to 108 and 360 on two lanelets each, which is the
    #: other population: a real 25-50% overshoot held for ~1-2 s, concentrated
    #: on private-road lanelets entered from faster roads. See
    #: MozartTest-Autoware reports/doppeltest_speeding_lanechange_findings.md.
    TOLERANCE = 0.10

    def __init__(self) -> None:
        self.map_parser = VectorMapParser.instance()
        self.lanelet_speed_limits = self.map_parser.get_attributes("speed_limit", float)
        self.min_speed_limit = min(self.lanelet_speed_limits.values())
        self.trace = list()

    def get_interested_topics(self) -> List[str]:
        return ["/localization/kinematic_state"]

    def on_new_message(self, topic: str, message, t):
        ego_position = message.pose.pose.position

        ego_velocity = calculate_velocity(message.twist.twist.linear) * 3.6

        route_ids = set()
        if hasattr(self, "oh") and self.oh:
            route_ids = self.oh.get_route_lanelet_ids()
        if route_ids:
            route_limits = [
                self.lanelet_speed_limits[lid]
                for lid in route_ids
                if lid in self.lanelet_speed_limits
            ]
            min_limit = min(route_limits) if route_limits else self.min_speed_limit
        else:
            min_limit = self.min_speed_limit

        if ego_velocity <= min_limit * (1 + SpeedingOracle.TOLERANCE):
            # cannot violate any speed limit
            self.trace.append((False, t, -1, dict()))
            return

        # Lanelets overlap, so a point is routinely inside more than one, and
        # the ego is speeding only if it exceeds the MOST PERMISSIVE limit it
        # is entitled to. Collect every containing lanelet and test the widest.
        #
        # This loop used to report the first containing lanelet whose limit was
        # exceeded and return. Because it tested the limit before accepting the
        # lanelet, a lanelet the ego was legally within was skipped and the
        # search continued until it found one to violate -- so wherever lanelets
        # overlapped the oracle attributed the ego to the STRICTEST of them and
        # could never conclude "within the limit of a lane it was also in".
        # Measured on run7: 16 of 108 findings above the 10% tolerance were the
        # ego at 6.55 km/h inside both lanelet 544 (limit 5) and lanelet 124253
        # (limit 8) -- 31% over one, 18% under the other, reported as speeding.
        point = BasicPoint2d(ego_position.x, ego_position.y)
        containing = []
        for lanelet in self.map_parser.lanelet_map.laneletLayer:
            if route_ids and lanelet.id not in route_ids:
                continue
            if inside(lanelet, point):
                lane_speed_limit = self.lanelet_speed_limits.get(lanelet.id)
                if lane_speed_limit is None:
                    continue
                containing.append((lane_speed_limit, lanelet.id))

        if containing:
            lane_speed_limit, lanelet_id = max(containing, key=lambda c: c[0])
            if ego_velocity > lane_speed_limit * (1 + SpeedingOracle.TOLERANCE):

                features = self.get_basic_info_from_localization(message)
                features["speed_limit"] = lane_speed_limit
                features["lanelet_id"] = lanelet_id
                # The comparison above is km/h against the map's km/h
                # speed_limit, but get_basic_info_from_localization stores
                # `speed` in m/s -- so a record reads "speed 2.28,
                # speed_limit 8.0" and looks like a false positive when it
                # is a real one (8.21 km/h over an 8 km/h limit). Record the
                # speed that was actually compared, next to what it was
                # compared against. `speed` keeps its unit and value: it is
                # this violation's key_label, and UnsafeLaneChangeOracle
                # tests it too.
                features["speed_kmh"] = round(ego_velocity, 2)

                self.trace.append((True, t, lane_speed_limit, features))

                return

        self.trace.append((False, t, -1, dict()))

    def get_result(self):
        violations = list()

        for k, v in groupby(self.trace, key=lambda x: (x[0], x[2])):
            traces = list(v)
            start_time = datetime.fromtimestamp(traces[0][1] / 1000000000)
            end_time = datetime.fromtimestamp(traces[-1][1] / 1000000000)
            delta_t = (end_time - start_time).total_seconds()

            if k[0]:
                features = dict(traces[0][3])
                features["duration"] = delta_t

                violations.append(
                    Violation("SpeedingOracle", features, str(features["speed"]))
                )

        return violations
