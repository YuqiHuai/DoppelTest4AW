# One DoppelTest vehicle: Autoware plus the receiver/sender and the scenario
# generator's Python dependencies.
#
# The image is PINNED BY DIGEST, not taken from a floating tag. This digest is
# the amd64 child of ghcr.io/autowarefoundation/autoware:universe-devel-humble-1.9.0,
# i.e. Autoware 0.52.0 -- the release the instrumented overlay is built against.
# `universe-devel-cuda-amd64`, which this file used to name, moves with upstream
# and is NOT 0.52.0: a receiver built on it would drive a different planner than
# the one the overlay instruments, and nothing in a running stack would say so.
FROM ghcr.io/autowarefoundation/autoware@sha256:3ead2d77c6d7a26d196b3319b3a7760aa97f795e792535a25dcb16cfae127e2d

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# NOT `ENV ROS_LOCALHOST_ONLY=1`, which this file used to set. This image ships
# a CYCLONEDDS_URI whose config already names the `lo` interface;
# ROS_LOCALHOST_ONLY then injects a second interface (address 127.0.0.1) and
# Cyclone refuses both -- "lo: the same interface may not be selected twice",
# "No network interface selected" -- so every rclpy node dies with
# "rcl node's rmw handle is invalid" and the receiver never reaches /health.
# Isolation between vehicles does not depend on it: each vehicle is its own
# container, hence its own network namespace, and with discovery pinned to `lo`
# no DDS traffic leaves it. See autoware_launch/cyclonedds.xml.

# receiver/sender: fastapi (uvicorn, httpx), the Apollo protobuf wire format,
# shapely for the violation analyser.
# generator: deap, networkx, shapely, requests -- installed here as well so
# test_main.py can run in a container of this image, which already has the
# lanelet2 Python bindings the map parser wants. Running it on the host instead
# needs `uv sync` and a ROS lanelet2 install.
RUN pip3 install --no-cache-dir \
      "fastapi[standard]" "protobuf==3.20.*" shapely \
      deap networkx requests

CMD ["bash", "-c", "exec bash"]
