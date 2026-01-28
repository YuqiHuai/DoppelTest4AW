FROM ghcr.io/autowarefoundation/autoware:universe-devel-cuda-amd64

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip \
    && python3 -m pip install --no-cache-dir "fastapi[standard]" "protobuf==3.20.*" shapely \
    && rm -rf /var/lib/apt/lists/*

CMD ["bash", "-c", "exec bash"]
