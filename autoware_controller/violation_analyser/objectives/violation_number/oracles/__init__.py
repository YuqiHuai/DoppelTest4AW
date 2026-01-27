from objectives.violation_number.oracles.OracleInterface import OracleInterface
from objectives.violation_number.oracles.OracleManager import OracleManager
from tools.autoware_tools.rosbag_reader import ROSBagReader
from typing import List

class RecordAnalyzer:
    record_path: str

    def __init__(
        self,
        record_path: str,
        oracles: List[OracleInterface],
        route_lanelet_ids: List[int] | None = None,
    ) -> None:
        self.oracle_manager = OracleManager()
        self.record_path = record_path
        self.reader = ROSBagReader(record_path)
        self.oracles = oracles
        self.register_oracles()
        if route_lanelet_ids:
            self.oracle_manager.set_route_lanelet_ids(route_lanelet_ids)

    def register_oracles(self):
        for o in self.oracles:
            self.oracle_manager.register_oracle(o)

    def topic_names(self):
        return [
            '/localization/acceleration',
            '/localization/kinematic_state',
            '/perception/object_recognition/objects',
            '/planning/scenario_planning/trajectory',
            '/planning/mission_planning/route',
            '/planning/path_candidate/lane_change_left',
            '/planning/path_candidate/lane_change_right',
            '/perception/traffic_light_recognition/traffic_signals'
        ]

    def analyze(self):
        reader = ROSBagReader(self.record_path)
        for topic, message, t in reader.read_messages():
            if topic in self.topic_names():
                msg = reader.deserialize_msg(message, topic)
                self.oracle_manager.on_new_message(topic, msg, t)

        return self.get_results()

    def get_results(self):
        return self.oracle_manager.get_results()
