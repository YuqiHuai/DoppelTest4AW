#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="$(cd "$(dirname "$0")" && pwd)"
START_SCRIPT="$BASE_DIR/autoware_launch/scripts/dev_start.sh"
PREFIX="autoware_"
CONTAINER_COUNT="${CONTAINER_COUNT:-5}"
AUTO_START_RECEIVER="${AUTO_START_RECEIVER:-1}"
RECEIVER_PORT="${RECEIVER_PORT:-5002}"
DOCKER_TIMEOUT="${DOCKER_TIMEOUT:-20}"

cd "$BASE_DIR"

is_running() {
  docker ps --format '{{.Names}}' | grep -Fxq "$1"
}

wait_for_running() {
  local cname="$1"
  for _ in {1..60}; do
    if is_running "$cname"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

container_ip() {
  docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$1"
}

receiver_is_healthy() {
  local cname="$1"
  docker exec "$cname" bash -lc "python3 -c \"import sys, urllib.request; urllib.request.urlopen('http://127.0.0.1:${RECEIVER_PORT}/health', timeout=2); sys.exit(0)\"" >/dev/null 2>&1
}

restart_receiver() {
  local cname="$1"
  local domain_id="$2"
  echo "[receiver] stopping old receiver in $cname"
  if ! timeout "$DOCKER_TIMEOUT" docker exec "$cname" bash -lc "pkill -f '[p]ython3 autoware_controller/receiver.py' >/dev/null 2>&1 || true"; then
    echo "[ERROR] timed out while stopping old receiver in $cname"
    return 1
  fi

  echo "[receiver] launching receiver in $cname"
  if ! timeout "$DOCKER_TIMEOUT" docker exec -d \
    -e ROS_DOMAIN_ID="$domain_id" \
    "$cname" \
    bash -lc "source /ros_entrypoint.sh && LOG_DIR=\"\$HOME/DoppelAutoware/container_${domain_id}/log\" && mkdir -p \"\$LOG_DIR\" && cd \"\$HOME/DoppelAutoware\" && exec python3 autoware_controller/receiver.py >> \"\$LOG_DIR/receiver_stdout.log\" 2>&1"; then
    echo "[ERROR] timed out while launching receiver in $cname"
    return 1
  fi

  echo "[receiver] launch command returned for $cname"
}

wait_for_receiver() {
  local cname="$1"
  for _ in {1..60}; do
    if receiver_is_healthy "$cname"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

show_receiver_failure() {
  local cname="$1"
  local domain_id="$2"
  local log_file="$BASE_DIR/container_${domain_id}/log/receiver_stdout.log"

  echo "[ERROR] receiver did not become healthy in $cname"
  if [[ -f "$log_file" ]]; then
    echo "[ERROR] Last receiver log lines from $log_file:"
    tail -n 50 "$log_file" || true
  else
    echo "[ERROR] receiver log file not found: $log_file"
  fi

  echo "[ERROR] Receiver process status in $cname:"
  docker exec "$cname" bash -lc "pgrep -af '[p]ython3 autoware_controller/receiver.py' || true" || true
}

declare -a RECEIVER_URLS

# Phase 1: ensure all containers are running.
for i in $(seq 1 "$CONTAINER_COUNT"); do
  cname="${PREFIX}${i}"

  if is_running "$cname"; then
    echo "[skip] $cname already running"
  else
    echo "[start] $cname"
    if ! bash "$START_SCRIPT" --use_multi_container --container_name "$cname" --ros-domain-id "$i"; then
      if is_running "$cname"; then
        echo "[warn] $cname is running, continue"
      else
        echo "[ERROR] failed to start $cname"
        exit 1
      fi
    fi
  fi

  if ! wait_for_running "$cname"; then
    echo "[ERROR] $cname not running after wait"
    exit 1
  fi
done

# Phase 2: launch all receivers.
for i in $(seq 1 "$CONTAINER_COUNT"); do
  cname="${PREFIX}${i}"
  if [[ "$AUTO_START_RECEIVER" != "0" ]]; then
    if receiver_is_healthy "$cname"; then
      echo "[receiver] already healthy in $cname"
      continue
    fi
    echo "[receiver] starting receiver for $cname"
    if ! restart_receiver "$cname" "$i"; then
      show_receiver_failure "$cname" "$i"
      exit 1
    fi
  fi
done

# Phase 3: wait for all receivers to become healthy.
for i in $(seq 1 "$CONTAINER_COUNT"); do
  cname="${PREFIX}${i}"
  if [[ "$AUTO_START_RECEIVER" != "0" ]]; then
    echo "[receiver] waiting for healthy receiver in $cname"
    if ! wait_for_receiver "$cname"; then
      show_receiver_failure "$cname" "$i"
      exit 1
    fi
    echo "[receiver] healthy: $cname"
  fi
done

# Phase 4: collect receiver URLs.
for i in $(seq 1 "$CONTAINER_COUNT"); do
  cname="${PREFIX}${i}"
  ip="$(container_ip "$cname")"
  if [[ -z "$ip" ]]; then
    echo "[ERROR] could not determine IP for $cname"
    exit 1
  fi
  RECEIVER_URLS+=("http://${ip}:${RECEIVER_PORT}")
done

echo
echo "Receiver URLs:"
for i in $(seq 1 "$CONTAINER_COUNT"); do
  echo "  ${PREFIX}${i}: ${RECEIVER_URLS[$((i-1))]}"
done
echo
echo "Next:"
echo "  python3 scenario_runner/test_main.py --num-vehicles $CONTAINER_COUNT --generations 1"
