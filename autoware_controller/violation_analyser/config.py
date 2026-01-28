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
