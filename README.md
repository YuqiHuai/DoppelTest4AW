# DoppelTest for Autoware

Implementation of **DoppelTest** for Autoware.

## Paper
The DoppelTest paper (ICSE 2023) is available at:

`https://dl.acm.org/doi/10.1109/ICSE48619.2023.00216`

## What This Repo Contains
- `autoware_controller/`: Receiver API, sender, and analysis tooling used to coordinate multiple Autoware instances.
- `autoware_launch/`: Docker/build/runtime helper scripts for Autoware containers.
- `scenario_runner/`: Scenario execution utilities.
- `start.sh`: Convenience script that opens a tmux session and attaches to 5 Autoware containers.

## Prerequisites
- Linux host with Docker installed and working.
- `tmux` (used by `start.sh`).
- `rocker` (used by `autoware_launch/scripts/dev_start.sh`).
- NVIDIA GPU + drivers (the container launcher uses `--nvidia`).
- X11 forwarding available on the host (the launcher uses `--x11`).
- ROS 2 / Autoware environment baked into the Docker image used by this repo.

Optional but commonly needed for setup scripts:
- `ansible-galaxy`, `ansible-playbook`
- `gdown`
- `unzip`

## Quick Start (Recommended)

### 1. Clone the repository
```bash
git clone <your-fork-or-repo-url>
cd DoppelAutoware
```

### 2. (Optional) Download map/data assets
This repo includes a setup script that downloads a sample map and Autoware data into `data/`.

```bash
bash autoware_launch/scripts/setup.sh
```

### 3. Build the multi-container image
`start.sh` expects the multi-container image flow.

```bash
bash autoware_launch/build/build_docker.sh --multi_container
```

The current default image name used by `autoware_launch/build/build_docker.sh --multi_container` is `doppel_autoware_multi_container:latest` (this is a script default in this repo, not a universal Docker/Autoware name).

### 4. Start and attach to 5 containers with `start.sh`
```bash
bash start.sh
```

Why 5 containers:
- The default multi-instance workflow in this repo is a **5-vehicle setup**.
- `start.sh` creates `autoware_1` ... `autoware_5` to match that experiment layout.
- The sender code also includes a default receiver cluster list with 5 peer IPs (`172.17.0.2` ... `172.17.0.6`).
- If you want fewer/more instances, use the manual container commands or modify `start.sh`.

Why `tmux`:
- You usually need multiple long-running shells at the same time (one per container).
- `tmux` keeps all container shells in one terminal session and makes it easier to monitor/restart them.
- `start.sh` uses one tmux pane per container so you can immediately run per-container commands (for example `export ROS_DOMAIN_ID=...` and `python3 autoware_controller/receiver.py`).

What `start.sh` does:
- Creates (or recreates) a tmux session named `autoware`.
- Opens 5 panes.
- Starts or reuses containers named `autoware_1` ... `autoware_5`.
- Enters each container in its corresponding pane.

Important:
- It **kills an existing tmux session named `autoware`** before recreating it.
- It does **not** automatically assign `ROS_DOMAIN_ID` values inside the shells.

## Per-Container Setup (Required)
After `bash start.sh`, each tmux pane is inside a different container shell.

Set a unique ROS domain per container so the ROS 2 graphs do not collide:

Pane for `autoware_1`:
```bash
export ROS_DOMAIN_ID=1
export RECEIVER_INSTANCE=1
cd ~/DoppelAutoware
```

Pane for `autoware_2`:
```bash
export ROS_DOMAIN_ID=2
export RECEIVER_INSTANCE=2
cd ~/DoppelAutoware
```

Repeat for all containers (e.g., `3`, `4`, `5`).

Notes:
- `autoware_launch/scripts/dev_into.sh` already enters the container with `/ros_entrypoint.sh` sourced.
- `RECEIVER_INSTANCE` is optional but recommended so logs are separated cleanly under `container_<id>/log/`.

## Running the Receiver (FastAPI + ROS node)
Inside each container where you want a receiver:

```bash
export ROS_DOMAIN_ID=<unique_id>
export RECEIVER_INSTANCE=<same_id>
python3 autoware_controller/receiver.py
```

Default receiver server:
- Host: `0.0.0.0`
- Port: `5002`

API URL note (important):
- `0.0.0.0` means the receiver listens on all interfaces **inside that container**.
- Use `127.0.0.1:5002` only when you run `curl` **inside the same container** as `receiver.py`.
- From another container (or from the host), use the target container's reachable IP/hostname, for example `http://<receiver-container-ip>:5002`.
- Replace all example receiver IPs/hostnames in commands with the actual addresses from your machine/container network.
- To discover a container IP, run `hostname -I` inside that container, or from the host run `docker inspect <container_name>` and check its network address.

Example (same container shell):
```bash
curl http://127.0.0.1:5002/health
```

Example (different container or host):
```bash
export RECEIVER_API_BASE="http://<receiver-container-ip>:5002"
curl "$RECEIVER_API_BASE/health"
```

## Starting Autoware and Sender via the Receiver API
Once `receiver.py` is running, you can control Autoware and the sender through HTTP endpoints.

Set the API base URL first (same-container or remote container):
```bash
# same container shell:
export RECEIVER_API_BASE="http://127.0.0.1:5002"

# or another container / host:
# export RECEIVER_API_BASE="http://<receiver-container-ip>:5002"
```

Start Autoware (uses defaults from `autoware_controller/receiver_app/config.py`):
```bash
curl -X POST "$RECEIVER_API_BASE/autoware/start" \
  -H 'Content-Type: application/json' \
  -d '{}'
```

Start sender:
```bash
curl -X POST "$RECEIVER_API_BASE/sender/start" \
  -H 'Content-Type: application/json' \
  -d '{}'
```

Set the sender peer receiver URLs for your actual container IPs/hostnames (do not rely on the example `172.17.0.2` ... `172.17.0.6` values unless your Docker network matches them):
```bash
curl -X POST "$RECEIVER_API_BASE/sender/start" \
  -H 'Content-Type: application/json' \
  -d '{
    "receiver_urls": [
      "http://<peer1-ip>:5002/perception",
      "http://<peer2-ip>:5002/perception"
    ]
  }'
```

Check status:
```bash
curl "$RECEIVER_API_BASE/autoware/status"
curl "$RECEIVER_API_BASE/sender/status"
```

## Running Experiments (Recommended: `scenario_runner/test_main.py`)
For actual multi-vehicle experiments, the normal entrypoint is:

`scenario_runner/test_main.py`

Use the manual API calls above for smoke tests/debugging. For experiment runs, `test_main.py` orchestrates the workflow across all receivers.

### What `test_main.py` does for you
Given a list of receiver URLs, the script will repeatedly generate scenarios and drive the receiver APIs to run them. It automatically:
- starts Autoware on active vehicles (via `/autoware/start`) if needed
- restarts sender processes with per-vehicle peer receiver URLs (via `/sender/restart`)
- initializes localization and route for each active vehicle
- switches vehicles into autonomous mode at scheduled times
- publishes pedestrians and traffic signals during the scenario
- starts/stops rosbag logging for active vehicles
- collects violations/decisions after each scenario
- writes per-scenario JSON results and GA selection logs
- attempts recovery/retry when autonomous mode is temporarily unavailable

### Before running `test_main.py` (important)
You still need to prepare the containers first:
1. Start containers (for example with `bash start.sh`).
2. In each container, set unique `ROS_DOMAIN_ID` (and preferably `RECEIVER_INSTANCE`).
3. Start `python3 autoware_controller/receiver.py` in each container that will participate.

`test_main.py` expects the receiver API to already be running on each vehicle endpoint.

### Where to run `test_main.py`
Run it in a shell that:
- can reach all receiver URLs over HTTP
- has access to this repo checkout (for the map path and local outputs)
- has the required Python dependencies used by the scenario runner (`deap`, `numpy`, `requests`, `shapely`, etc.)

This is often the host machine (repo root), but any environment with network access to the receivers and the needed Python packages can work.

### Minimal experiment command (explicit URLs recommended)
From the repo root:

```bash
python3 scenario_runner/test_main.py \
  --url http://<vehicle1-receiver-ip>:5002 \
  --url http://<vehicle2-receiver-ip>:5002 \
  --url http://<vehicle3-receiver-ip>:5002 \
  --url http://<vehicle4-receiver-ip>:5002 \
  --url http://<vehicle5-receiver-ip>:5002 \
  --num-vehicles 5 \
  --generations 1 \
  --log-dir scenario_runs
```

Use your actual receiver addresses. Do not copy the example IP placeholders as-is.

### Alternative: provide URLs through environment variable
Instead of repeating `--url`, you can export a comma-separated list:

```bash
export AUTOWARE_RECEIVER_URLS="http://<v1>:5002,http://<v2>:5002,http://<v3>:5002,http://<v4>:5002,http://<v5>:5002"
python3 scenario_runner/test_main.py --num-vehicles 5 --generations 1
```

### Common run modes
Fixed vehicle count (same number of vehicles in every scenario):
```bash
python3 scenario_runner/test_main.py \
  --url http://<v1>:5002 \
  --url http://<v2>:5002 \
  --url http://<v3>:5002 \
  --num-vehicles 3 \
  --generations 10
```

Mixed-size scenarios (vehicle count varies per scenario):
```bash
python3 scenario_runner/test_main.py \
  --url http://<v1>:5002 \
  --url http://<v2>:5002 \
  --url http://<v3>:5002 \
  --url http://<v4>:5002 \
  --url http://<v5>:5002 \
  --min-vehicles 2 \
  --max-vehicles 5 \
  --duration-hours 1
```

Conflict-only generation is enabled by default. To allow general scenarios:
```bash
python3 scenario_runner/test_main.py ... --no-conflict-only
```

### Key options in `test_main.py`
- `--url` (repeatable): receiver base URLs (recommended)
- `--num-vehicles`: fixed vehicle count for all scenarios
- `--min-vehicles` / `--max-vehicles`: mixed-size scenarios
- `--map`: Lanelet2 OSM path (default is `autoware_map/sample-map-planning/lanelet2_map.osm`)
- `--population`: GA population size
- `--generations`: generation-limited run (`0` means use time limit)
- `--duration-hours`: time-limited run when `--generations 0`
- `--log-dir`: output directory for experiment metadata/logs (default `scenario_runs`)
- `--restart-wait`: wait time after Autoware restart before recovery retry
- `--max-recovery-retries`: retries before marking a scenario failed

### What outputs to expect
`test_main.py` writes experiment metadata to the `--log-dir` directory (default `scenario_runs`):
- `test_main.log`: main runner stdout/stderr tee log
- `Generation_XXXXX_Scenario_XXXXX.json`: per-scenario result record (scenario config, violations, decisions, fitness, status)
- `GA_selection_gen_XXXXX.json`: selected individuals after each generation

During each scenario, the runner also requests rosbag recording through the receiver API, so per-vehicle runtime logs/bags are typically created under:
- `container_<id>/log/record_log/` (when `RECEIVER_INSTANCE` is set)

### Common setup mistakes (and how to avoid them)
- Receiver not running:
  Start `python3 autoware_controller/receiver.py` in every container before launching `test_main.py`.
- Wrong receiver URLs:
  `test_main.py` needs the receiver API base URL (for example `http://<ip>:5002`), not the `/perception` endpoint URL.
- Wrong network addresses:
  Replace all example IPs with real container/host-reachable addresses from your environment.
- Reused `ROS_DOMAIN_ID` values:
  Each Autoware container should use a unique ROS domain for multi-vehicle runs.
- Not enough endpoints for your configured vehicle count:
  If `--max-vehicles 5`, provide at least 5 receiver URLs (via `--url` or `AUTOWARE_RECEIVER_URLS`).
- Missing map/data assets:
  Ensure the default map path exists or pass `--map` explicitly.

## Manual Container Workflow (Without `start.sh`)
If you prefer manual control:

```bash
# Start one container
bash autoware_launch/scripts/dev_start.sh --use_multi_container --container_name autoware_1

# Enter that container
bash autoware_launch/scripts/dev_into.sh --container_name autoware_1
```

Then set:
```bash
export ROS_DOMAIN_ID=1
export RECEIVER_INSTANCE=1
```

## Logs and Outputs
- Receiver logs / Autoware logs / recording outputs are typically written under `container_<id>/log/` (when `RECEIVER_INSTANCE` is set).
- Scenario runner outputs are written under the `--log-dir` path (default `scenario_runs`).
- Analysis outputs may also be written under repo-local directories such as `analyse/` depending on the script used.

## Publish/Reuse Notes (Current Assumptions)
This repository currently includes scripts that are convenient for local use but may need adjustment for other machines:
- `autoware_launch/scripts/dev_start.sh` expects Docker + rocker + GPU/X11 support.
- The Docker image names default to `doppel_autoware*` (configurable by environment variables).
- The default `autoware_data` mount path is derived from `<workspace>/data/autoware_data` (or `AUTOWARE_DATA_HOST_PATH` if set).
- `autoware_launch/scripts/dev_start.sh` uses `autoware_dev_${USER}` as its default container name when `--container_name` is not provided. This is dynamic, so it becomes `autoware_dev_<your_host_username>`.
- Sender/runner defaults include Docker-bridge example receiver IPs (`172.17.0.2` ... `172.17.0.6`). These are examples for one common Docker network layout and may need `--url` / `receiver_urls` overrides on another machine.

## Citation
If you use this repo in research, please cite the DoppelTest paper above and describe any local changes to Autoware or map assets used for reproduction.
