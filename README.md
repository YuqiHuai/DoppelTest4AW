# DoppelTest for Autoware

Autoware implementation of DoppelTest for multi-vehicle scenario generation and execution.

Paper: `https://dl.acm.org/doi/10.1109/ICSE48619.2023.00216`

## Repo Layout
- `autoware_controller/`: receiver API, sender, rosbag control, violation analysis
- `autoware_launch/`: Docker image and container launch helpers
- `autoware_map/`: Lanelet2 map directories used by Autoware experiments
- `scenario_runner/`: scenario generation and experiment orchestration
- `start.sh`: starts containers and receivers for the default multi-vehicle workflow

## Requirements
- Linux
- Docker
- `rocker`
- NVIDIA GPU, drivers, and X11 support for the provided launch scripts

## Python and `rocker` Setup
Install `uv` first. Official docs: `https://docs.astral.sh/uv/getting-started/installation/`

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

From the repo root, sync the project environment:

```bash
uv sync
```

This repo already vendors `rocker` under `autoware_launch/rocker/`. `uv sync` installs it from the local path, so do not clone it separately.

If you want to run on maps other than the default `BorregasAve`, install ROS 2 first using the official Humble installation guide:

- `https://docs.ros.org/en/humble/Installation.html`

Then source ROS 2 and install Lanelet2:

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-lanelet2
```

`lanelet2` is not managed by `uv` here. It comes from the ROS environment and is used for reliable Lanelet2 routing/regulatory parsing and by the violation analyzer.

## Default Workflow
### 1. Build the multi-container image
```bash
bash autoware_launch/build/build_docker.sh --multi_container
```

### 2. Start containers and receivers
```bash
bash start.sh
```

This starts or reuses `autoware_1` to `autoware_5`, assigns `ROS_DOMAIN_ID=1..5`, launches `autoware_controller/receiver.py` in the background, waits for `/health`, and prints the receiver URLs.

### 3. Run one GA iteration with 5 vehicles
```bash
uv run --script scenario_runner/test_main.py --num-vehicles 5 --generations 1
```

Meaning:
- `--num-vehicles 5`: each generated scenario uses 5 active Autoware vehicles
- `--generations 1`: run one GA generation

After experiments, remove the Autoware containers:

```bash
bash autoware_launch/scripts/kill_all.sh
```

By default, `test_main.py` auto-discovers running containers named `autoware_1`, `autoware_2`, ... through Docker and uses their current container IPs.

Default map:
- `autoware_map/BorregasAve/lanelet2_map.osm`

To run on `sample-map-planning`, add `--map`:
```bash
uv run --script scenario_runner/test_main.py \
  --num-vehicles 5 \
  --generations 1 \
  --map autoware_map/sample-map-planning/lanelet2_map.osm
```

## `test_main.py`
Main entrypoint:

```bash
uv run --script scenario_runner/test_main.py
```

It orchestrates:
- Autoware startup and recovery
- sender restart with peer receiver URLs
- localization and route initialization
- autonomous mode switching
- pedestrian and traffic signal publishing
- rosbag control
- violation, decision, and fitness collection

Common examples:

Fixed vehicle count:
```bash
uv run --script scenario_runner/test_main.py \
  --num-vehicles 3 \
  --generations 10
```

Mixed-size scenarios:
```bash
uv run --script scenario_runner/test_main.py \
  --min-vehicles 2 \
  --max-vehicles 5 \
  --duration-hours 1
```

Explicit receiver URLs:
```bash
uv run --script scenario_runner/test_main.py \
  --url http://<vehicle1-ip>:5002 \
  --url http://<vehicle2-ip>:5002 \
  --url http://<vehicle3-ip>:5002 \
  --num-vehicles 3
```

Useful flags:
- `--log-dir`: output directory for scenario JSON files and GA logs
- `--restart-wait`: wait time after Autoware restart, default `60`
- `--max-recovery-retries`: per-scenario recovery attempts before marking failure
- `--conflict-only`: generate only conflict scenarios
- `--no-conflict-only`: disable conflict-only generation

## Receiver API
The receiver listens on `0.0.0.0:5002` inside each container.

Health check:
```bash
curl http://127.0.0.1:5002/health
```

Start Autoware:
```bash
curl -X POST http://127.0.0.1:5002/autoware/start \
  -H 'Content-Type: application/json' \
  -d '{}'
```

Start sender:
```bash
curl -X POST http://127.0.0.1:5002/sender/start \
  -H 'Content-Type: application/json' \
  -d '{}'
```

Start sender with explicit peer URLs:
```bash
curl -X POST http://127.0.0.1:5002/sender/start \
  -H 'Content-Type: application/json' \
  -d '{
    "receiver_urls": [
      "http://<peer1-ip>:5002/perception",
      "http://<peer2-ip>:5002/perception"
    ]
  }'
```

## Runtime Parameters
Main parameters still relevant in the current code:
- `ROS_DOMAIN_ID`: per-container ROS 2 isolation; assigned automatically by `start.sh`, `dev_start.sh`, and `dev_into.sh`
- `CONTAINER_COUNT`: number of containers started by `start.sh`, default `5`
- `AUTO_START_RECEIVER`: set `0` to make `start.sh` skip receiver startup
- `AUTOWARE_RECEIVER_URLS`: explicit receiver base URLs for `test_main.py`
- `AUTOWARE_RECEIVER_CONTAINER_PREFIX`: Docker auto-discovery prefix for `test_main.py`, default `autoware_`
- `AUTOWARE_RECEIVER_PORT`: receiver port for Docker auto-discovery, default `5002`
- `RECEIVER_LOG_ROOT`: override receiver log directory

Removed:
- `RECEIVER_INSTANCE`

## Outputs
`test_main.py` writes to the selected `--log-dir`:
- `test_main.log`
- `Generation_XXXXX_Scenario_XXXXX.json`
- `GA_selection_gen_XXXXX.json`

Receiver logs default to:
```text
container_<ROS_DOMAIN_ID>/log/
```

## Manual Control
Start one container:
```bash
bash autoware_launch/scripts/dev_start.sh --use_multi_container --container_name autoware_1
```

Enter a container shell:
```bash
bash autoware_launch/scripts/dev_into.sh --container_name autoware_1
```

Enter and start the receiver:
```bash
bash autoware_launch/scripts/dev_into.sh --container_name autoware_1 --start-receiver
```

For custom container names without numeric suffixes:
```bash
bash autoware_launch/scripts/dev_start.sh --use_multi_container --container_name my_av --ros-domain-id 7
bash autoware_launch/scripts/dev_into.sh --container_name my_av --ros-domain-id 7 --start-receiver
```

## Common Mistakes
- Passing `/perception` URLs to `test_main.py` instead of receiver base URLs
- Running `test_main.py` from an environment that cannot access Docker when relying on auto-discovery
- Using a `--map` path that exists on the host but not inside the Autoware containers

## Citation
If you use this repository in research, cite the DoppelTest paper and document any local changes to Autoware, maps, or analysis logic.
