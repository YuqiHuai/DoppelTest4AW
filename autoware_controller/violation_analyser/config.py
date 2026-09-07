from pathlib import Path

"""
Global configurations
"""

# DIRECTORIES
PROJECT_ROOT = Path(__file__).resolve().parent
REPO_ROOT = PROJECT_ROOT.parents[1]
ADS_MAP_DIR = str(REPO_ROOT / "autoware_map")  # Autoware maps (.osm) live here
ADS_RECORD_DIR = f"{PROJECT_ROOT}/data/records"
TMP_RECORDS_DIR = f'/tmp/scenario_test_runner'

# VEHICLE CONFIGS FOR AUTOWARE
AUTOWARE_VEHICLE_LENGTH = 4.77
AUTOWARE_VEHICLE_WIDTH = 1.83
AUTOWARE_VEHICLE_HEIGHT = 2.5
AUTOWARE_VEHICLE_back_edge_to_center = 1.030
# Distance from the localization pose (base_link, on the rear axle) to the
# FRONT AXLE. Autoware's own vehicle_info for this stack gives wheel_base 2.74
# and front_overhang 1.0, i.e. the front bumper is 3.74 m ahead of the pose --
# the same number AUTOWARE_VEHICLE_LENGTH - back_edge_to_center gives here.
AUTOWARE_VEHICLE_WHEEL_BASE = 2.74
