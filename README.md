# DoppelTest for Autoware

Implementation of **DoppelTest** for Autoware.

## Paper
The DoppelTest paper (ICSE 2023) is available at:
```
https://dl.acm.org/doi/10.1109/ICSE48619.2023.00216
```

## Repository layout
- `autoware_controller/` : Autoware integration and control utilities
- `README.md` : Build/run/evaluation instructions for this repo

## Prerequisites

🚧**WIP**🚧

## Setup
1. Clone this repository into your workspace.
2. Source your Autoware/ROS 2 environment.
3. Install any Python dependencies used by `autoware_controller/`.



## Run
Describe the launch/execute steps to:
1. Start Autoware
2. Start DoppelTest components
3. Start the receiver/controller

Autoware containers:
```
# Build multi-container image (once, or after Dockerfile changes)
bash autoware_launch/build/build_docker.sh --multi_container

# Start containers (repeat for each instance)
bash autoware_launch/scripts/dev_start.sh --use_multi_container --container_name autoware_1
bash autoware_launch/scripts/dev_start.sh --use_multi_container --container_name autoware_2

# Enter a container
bash autoware_launch/scripts/dev_into.sh --container_name autoware_1
```

Example placeholders:
```
# Terminal 1
<autoware launch command>

# Terminal 2
python3 autoware_controller/receiver.py
```

## Evaluation
Describe how to execute the DoppelTest evaluation, including:
- Test scenarios or routes
- Input datasets
- Expected outputs (logs, metrics, reports)

Example placeholders:
```
# Run an evaluation scenario
<evaluation command>

# Collect results
<results collection command>
```

## Results
Summarize where results are written and how to interpret them.

## Notes
If your environment has special requirements (GPU, maps, vehicle model, etc.),
document them here.
