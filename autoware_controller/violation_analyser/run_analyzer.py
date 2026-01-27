import argparse
import sys
from pathlib import Path
from typing import List, Optional

# Ensure the local violation_analyser package directory is on sys.path so that
# absolute imports such as `environment.MapLoader` resolve even when this file
# is imported from outside the package root (e.g., from receiver.py).
_PACKAGE_ROOT = Path(__file__).resolve().parent
if _PACKAGE_ROOT.exists():
    sys.path.insert(0, str(_PACKAGE_ROOT))

from environment.MapLoader import MapLoader
from objectives.violation_number.oracles import RecordAnalyzer
from objectives.violation_number.oracles.impl.ComfortOracle import ComfortOracle
from objectives.violation_number.oracles.impl.SpeedingOracle import SpeedingOracle
from objectives.violation_number.oracles.impl.CollisionOracle import CollisionOracle
from objectives.violation_number.oracles.impl.JunctionLaneChangeOracle import (
    JunctionLaneChangeOracle,
)
from objectives.violation_number.oracles.impl.ModuleDelayOracle import ModuleDelayOracle
from objectives.violation_number.oracles.impl.ModuleOracle import ModuleOracle
from objectives.violation_number.oracles.impl.UnsafeLaneChangeOracle import (
    UnsafeLaneChangeOracle,
)
from objectives.violation_number.oracles.impl.TrafficSignalOracle import (
    TrafficSignalOracle,
)


def measure_violations(
    map_name, scenario_record_path, route_lanelet_ids: Optional[List[int]] = None
):
    MapLoader(map_name)

    target_oracles = [
        ComfortOracle(),
        CollisionOracle(),
        SpeedingOracle(),
        ModuleDelayOracle(),
        ModuleOracle(),
        UnsafeLaneChangeOracle(),
        JunctionLaneChangeOracle(),
        TrafficSignalOracle(),
    ]

    analyzer = RecordAnalyzer(
        record_path=str(scenario_record_path),
        oracles=target_oracles,
        route_lanelet_ids=route_lanelet_ids,
    )
    violations = analyzer.analyze()
    print(
        f"Violation Results: {[(violation.main_type, violation.key_label) for violation in violations]}"
    )
    return violations


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Analyze an Autoware scenario record for rule violations."
    )
    parser.add_argument(
        "record_path",
        type=Path,
        help="Path to the scenario record directory or .db3 file.",
    )
    parser.add_argument(
        "--route-ids",
        default="",
        help="Comma-separated lanelet IDs to constrain SpeedingOracle.",
    )
    args = parser.parse_args()

    # Accept either a directory that contains a .db3 file or a .db3 file itself.
    record_path = args.record_path
    if record_path.is_file():
        record_path = record_path.parent
    elif not record_path.is_dir():
        raise FileNotFoundError(f"{record_path} is not a file or directory.")

    # Map selection is fixed to the bundled sample map.
    map_name = "sample-map-planning"
    route_ids = [
        int(v) for v in args.route_ids.split(",") if v.strip().isdigit()
    ] or None
    violations = measure_violations(map_name, record_path, route_ids)
    print(len(violations))
