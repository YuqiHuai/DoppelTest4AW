#!/usr/bin/env bash
set -euo pipefail

SESSION="autoware"
BASE_DIR="$(cd "$(dirname "$0")" && pwd)"
START_SCRIPT="$BASE_DIR/autoware_launch/scripts/dev_start.sh"
INTO_SCRIPT="$BASE_DIR/autoware_launch/scripts/dev_into.sh"
PREFIX="autoware_"

cd "$BASE_DIR"

is_running() {
  docker ps --format '{{.Names}}' | grep -Fxq "$1"
}

# Recreate the tmux session
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
tmux new-session -d -s "$SESSION" -c "$BASE_DIR"

# Create 5 panes and record their pane IDs
declare -a PANES
PANES+=("$(tmux display-message -p -t "$SESSION:0.0" '#{pane_id}')")
for _ in {2..5}; do
  PANES+=("$(tmux split-window -h -t "$SESSION:0" -c "$BASE_DIR" -P -F '#{pane_id}')")
  tmux select-layout -t "$SESSION:0" even-horizontal
done

# Start or reuse 5 containers (skip ones that are already running)
for i in {1..5}; do
  cname="${PREFIX}${i}"

  if is_running "$cname"; then
    echo "[skip] $cname already running"
    continue
  fi

  echo "[start] $cname"
  if ! bash "$START_SCRIPT" --use_multi_container --container_name "$cname"; then
    # Some scripts may return non-zero even when startup succeeded, so re-check container state.
    if is_running "$cname"; then
      echo "[warn] $cname is running, continue"
    else
      echo "[ERROR] failed to start $cname"
      exit 1
    fi
  fi
done

# Enter the corresponding container in each pane
for i in {1..5}; do
  cname="${PREFIX}${i}"

  for _ in {1..60}; do
    is_running "$cname" && break
    sleep 1
  done

  if ! is_running "$cname"; then
    echo "[ERROR] $cname not running after wait"
    continue
  fi

  pane="${PANES[$((i-1))]}"
  tmux send-keys -t "$pane" "bash \"$INTO_SCRIPT\" --container_name \"$cname\"" C-m
done

tmux attach -t "$SESSION"
