#!/usr/bin/env bash
set -euo pipefail

PREFIX="${CONTAINER_PREFIX:-autoware_}"

if command -v rg >/dev/null 2>&1; then
  mapfile -t CONTAINERS < <(docker ps -a --format '{{.Names}}' | rg "^${PREFIX}[0-9]+$")
else
  mapfile -t CONTAINERS < <(docker ps -a --format '{{.Names}}' | grep -E "^${PREFIX}[0-9]+$")
fi

if [[ ${#CONTAINERS[@]} -eq 0 ]]; then
  echo "[cleanup] no matching containers found for prefix '${PREFIX}'"
  exit 0
fi

echo "[cleanup] removing containers:"
printf '  %s\n' "${CONTAINERS[@]}"
docker rm -f "${CONTAINERS[@]}"
echo "[cleanup] done"
