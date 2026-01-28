"""
Decision counter for Autoware rosbag (.db3) recordings.

Given a rosbag directory, this module counts the occurrences of several planning-
related decisions (driving direction, lane-change behaviors, velocity-factor
behaviors, and Autoware state transitions).
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import sys

from autoware_internal_planning_msgs.msg import PlanningFactor, PlanningFactorArray
from autoware_adapi_v1_msgs.msg import VelocityFactorArray
from autoware_system_msgs.msg import AutowareState

# Reuse the bag reader from the local violation_analyser package.
VIOLATION_ANALYSER_ROOT = Path(__file__).resolve().parent / "violation_analyser"
if VIOLATION_ANALYSER_ROOT.exists():
    sys.path.insert(0, str(VIOLATION_ANALYSER_ROOT))

from violation_analyser.tools.autoware_tools.rosbag_reader import ROSBagReader  # noqa: E402


BEHAVIOR_TOPIC = "/planning/planning_factors/behavior_path_planner"
LANE_CHANGE_LEFT_TOPIC = "/planning/planning_factors/lane_change_left"
LANE_CHANGE_RIGHT_TOPIC = "/planning/planning_factors/lane_change_right"
VELOCITY_TOPIC = "/api/planning/velocity_factors"
AUTOWARE_STATE_TOPIC = "/autoware/state"

PLANNING_BEHAVIOR_LABELS = {
    PlanningFactor.UNKNOWN: "UNKNOWN",
    PlanningFactor.NONE: "NONE",
    PlanningFactor.SLOW_DOWN: "SLOW_DOWN",
    PlanningFactor.STOP: "STOP",
    PlanningFactor.SHIFT_LEFT: "SHIFT_LEFT",
    PlanningFactor.SHIFT_RIGHT: "SHIFT_RIGHT",
    PlanningFactor.TURN_LEFT: "TURN_LEFT",
    PlanningFactor.TURN_RIGHT: "TURN_RIGHT",
}

AUTOWARE_STATE_LABELS = {
    AutowareState.INITIALIZING: "INITIALIZING",
    AutowareState.WAITING_FOR_ROUTE: "WAITING_FOR_ROUTE",
    AutowareState.PLANNING: "PLANNING",
    AutowareState.WAITING_FOR_ENGAGE: "WAITING_FOR_ENGAGE",
    AutowareState.DRIVING: "DRIVING",
    AutowareState.ARRIVED_GOAL: "ARRIVED_GOAL",
    AutowareState.FINALIZING: "FINALIZING",
}


@dataclass
class DecisionSummary:
    driving_direction: Dict[str, int]
    lane_change_left: Dict[str, int]
    lane_change_right: Dict[str, int]
    velocity_behaviors: Dict[str, int]
    autoware_states: Dict[str, int]

    def to_dict(self) -> Dict[str, Dict[str, int]]:
        return {
            "driving_direction": dict(self.driving_direction),
            "lane_change_left": dict(self.lane_change_left),
            "lane_change_right": dict(self.lane_change_right),
            "velocity_behaviors": dict(self.velocity_behaviors),
            "autoware_states": dict(self.autoware_states),
        }


def _resolve_record_path(record_path: Path) -> Path:
    path = Path(record_path).expanduser().resolve()
    if path.is_file():
        return path.parent
    if not path.is_dir():
        raise FileNotFoundError(f"{record_path} is not a valid bag directory or file.")
    return path


def _planning_behavior_name(value: int) -> str:
    return PLANNING_BEHAVIOR_LABELS.get(value, f"UNKNOWN_BEHAVIOR_{value}")


def _state_name(value: int) -> str:
    return AUTOWARE_STATE_LABELS.get(value, f"UNKNOWN_STATE_{value}")


def summarize_decisions(record_path: Path) -> DecisionSummary:
    """
    Parse a rosbag directory and aggregate all relevant decision counters.
    """
    bag_dir = _resolve_record_path(record_path)
    reader = ROSBagReader(str(bag_dir))

    driving_direction = Counter()
    lane_change_left = Counter()
    lane_change_right = Counter()
    velocity_behaviors = Counter()
    autoware_states = Counter()

    for topic, data, _ in reader.read_messages():
        if topic == BEHAVIOR_TOPIC:
            msg = reader.deserialize_msg(data, topic)
            if isinstance(msg, PlanningFactorArray):
                for factor in msg.factors:
                    driving_direction[
                        "forward" if factor.is_driving_forward else "reverse"
                    ] += 1
        elif topic == LANE_CHANGE_LEFT_TOPIC:
            msg = reader.deserialize_msg(data, topic)
            if isinstance(msg, PlanningFactorArray):
                for factor in msg.factors:
                    lane_change_left[_planning_behavior_name(factor.behavior)] += 1
        elif topic == LANE_CHANGE_RIGHT_TOPIC:
            msg = reader.deserialize_msg(data, topic)
            if isinstance(msg, PlanningFactorArray):
                for factor in msg.factors:
                    lane_change_right[_planning_behavior_name(factor.behavior)] += 1
        elif topic == VELOCITY_TOPIC:
            msg = reader.deserialize_msg(data, topic)
            if isinstance(msg, VelocityFactorArray):
                for factor in msg.factors:
                    label = factor.behavior if factor.behavior else "UNKNOWN"
                    velocity_behaviors[label] += 1
        elif topic == AUTOWARE_STATE_TOPIC:
            msg = reader.deserialize_msg(data, topic)
            if isinstance(msg, AutowareState):
                autoware_states[_state_name(msg.state)] += 1

    return DecisionSummary(
        driving_direction=driving_direction,
        lane_change_left=lane_change_left,
        lane_change_right=lane_change_right,
        velocity_behaviors=velocity_behaviors,
        autoware_states=autoware_states,
    )


def _cli() -> None:
    import argparse
    import json

    parser = argparse.ArgumentParser(
        description="Summarize Autoware planning decisions from a rosbag directory."
    )
    parser.add_argument(
        "record_path", type=Path, help="Path to a rosbag directory or db3 file."
    )
    args = parser.parse_args()

    summary = summarize_decisions(args.record_path)
    print(json.dumps(summary.to_dict(), indent=2))


if __name__ == "__main__":
    _cli()
