# DoppelTest for Autoware

Autoware implementation of DoppelTest for multi-vehicle scenario generation and execution.

Paper: `https://dl.acm.org/doi/10.1109/ICSE48619.2023.00216`

## Repo Layout
- `autoware_controller/`: receiver API, sender, rosbag control, violation analysis
- `autoware_launch/`: Docker image and container launch helpers
- `autoware_map/`: Lanelet2 map assets used by Autoware experiments
- `scenario_runner/`: scenario generation and experiment orchestration
- `start.sh`: opens a 5-pane `tmux` session and enters 5 Autoware containers

## Requirements
- Linux
- Docker
- `tmux`
- `rocker`
- NVIDIA GPU, drivers, and X11 support for the provided launch scripts

Optional setup tools:
- `ansible-galaxy`
- `ansible-playbook`
- `gdown`
- `unzip`

## Quick Start
### 1. Build the container image
```bash
bash autoware_launch/build/build_docker.sh --multi_container
```

### 2. Start the default 5-container session
```bash
bash start.sh
```

`start.sh` recreates a `tmux` session named `autoware`, opens 5 panes, and enters containers `autoware_1` to `autoware_5`.

### 3. In each container, set a unique ROS domain and start the receiver
Container 1:
```bash
export ROS_DOMAIN_ID=1
cd ~/DoppelAutoware
python3 autoware_controller/receiver.py
```

Container 2:
```bash
export ROS_DOMAIN_ID=2
cd ~/DoppelAutoware
python3 autoware_controller/receiver.py
```

Repeat for the remaining containers.

## Receiver API
The receiver listens on `0.0.0.0:5002` inside each container.

Use `127.0.0.1:5002` only from the same container. From another container or the host, use the receiver container's reachable IP.

Example health check:
```bash
curl http://127.0.0.1:5002/health
```

If you want shorter `curl` commands, you can define a shell helper:
```bash
export RECEIVER_API_BASE="http://127.0.0.1:5002"
curl "$RECEIVER_API_BASE/health"
```

`RECEIVER_API_BASE` is only a shell variable for examples in this README. It is not used by the project code.

### Start Autoware through the API
```bash
curl -X POST http://127.0.0.1:5002/autoware/start \
  -H 'Content-Type: application/json' \
  -d '{}'
```

### Start the sender
```bash
curl -X POST http://127.0.0.1:5002/sender/start \
  -H 'Content-Type: application/json' \
  -d '{}'
```

### Start the sender with explicit peer URLs
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

## Running Experiments
Main entrypoint:

```bash
python3 scenario_runner/test_main.py
```

What it does:
- starts Autoware on active vehicles
- restarts sender processes with peer receiver URLs
- initializes localization and routes
- switches vehicles to autonomous mode at scheduled times
- publishes pedestrians and traffic signals
- starts and stops rosbag logging
- collects violations, decisions, and fitness

### Before running `test_main.py`
1. Start the containers.
2. In each container, set a unique `ROS_DOMAIN_ID`.
3. Start `python3 autoware_controller/receiver.py` in each participating container.

`test_main.py` expects each receiver API to already be running.

### Minimal example
```bash
python3 scenario_runner/test_main.py \
  --url http://<vehicle1-ip>:5002 \
  --url http://<vehicle2-ip>:5002 \
  --url http://<vehicle3-ip>:5002 \
  --url http://<vehicle4-ip>:5002 \
  --url http://<vehicle5-ip>:5002 \
  --num-vehicles 5 \
  --generations 1 \
  --log-dir scenario_runs
```

### Alternative: receiver URLs from environment
```bash
export AUTOWARE_RECEIVER_URLS="http://<v1>:5002,http://<v2>:5002,http://<v3>:5002,http://<v4>:5002,http://<v5>:5002"
python3 scenario_runner/test_main.py --num-vehicles 5 --generations 1
```

### Common run modes
Fixed vehicle count:
```bash
python3 scenario_runner/test_main.py \
  --url http://<v1>:5002 \
  --url http://<v2>:5002 \
  --url http://<v3>:5002 \
  --num-vehicles 3 \
  --generations 10
```

Mixed-size scenarios:
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

Allow non-conflict scenarios:
```bash
python3 scenario_runner/test_main.py ... --no-conflict-only
```

### Important options
- `--url`: receiver base URL; repeat once per vehicle
- `--num-vehicles`: fixed vehicle count
- `--min-vehicles`, `--max-vehicles`: mixed-size runs
- `--map`: Lanelet2 map path
- `--population`: GA population size
- `--generations`: number of generations
- `--duration-hours`: time-limited run when `--generations 0`
- `--log-dir`: output directory for experiment results
- `--restart-wait`: wait after Autoware restart
- `--max-recovery-retries`: retry limit before failure

## Runtime Parameters
Parameters that matter in the current code:
- `ROS_DOMAIN_ID`: required; isolates each Autoware instance
- `RECEIVER_LOG_ROOT`: optional; overrides the receiver log directory
- `AUTOWARE_RECEIVER_URLS`: optional; comma-separated receiver base URLs for `test_main.py`
- `AUTOWARE_RECEIVER_URL`: optional; default receiver URL for helper code in `scenario_runner/framework/scenario/ad_agents.py`
- `RECEIVER_URLS`, `RECEIVER_URL`, `RECEIVER_CLUSTER_HOSTS`, `RECEIVER_PORT`, `RECEIVER_PATH`, `SELF_IP`: optional sender-side transport settings

Parameters removed from the current implementation:
- `RECEIVER_INSTANCE`: no longer used

## Outputs
`test_main.py` writes to the selected `--log-dir`:
- `test_main.log`
- `Generation_XXXXX_Scenario_XXXXX.json`
- `GA_selection_gen_XXXXX.json`

Receiver-side logs default to:
```text
container_<ROS_DOMAIN_ID>/log/
```

`RECEIVER_LOG_ROOT` overrides that path if needed.

## Manual Container Workflow
If you do not want to use `start.sh`:

```bash
bash autoware_launch/scripts/dev_start.sh --use_multi_container --container_name autoware_1
bash autoware_launch/scripts/dev_into.sh --container_name autoware_1
export ROS_DOMAIN_ID=1
```

## Common Mistakes
- Reusing the same `ROS_DOMAIN_ID` across containers
- Passing `/perception` URLs to `test_main.py` instead of receiver base URLs
- Using Docker example IPs without checking the actual container network
- Providing fewer receiver URLs than the configured vehicle count

## Notes
- The default workflow assumes 5 vehicles.
- Sender defaults include Docker bridge example IPs `172.17.0.2` to `172.17.0.6`.
- The provided launch scripts assume the local Docker/X11/GPU setup used by this project.

## Citation
If you use this repository in research, cite the DoppelTest paper and document any local changes to Autoware, maps, or analysis logic.
