# !/bin/bash

CONTAINER_NAME="autoware_dev_${USER}"
USE_MULTI_CONTAINER=false
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WORKSPACE_PATH=""
DATA_PATH=""

show_help() {
    echo -e "\e[34mUsage:\e[0m $0 [options]"
    echo -e "\e[34mOptions:\e[0m"
    echo -e "  \e[32m--container_name <name>\e[0m    Set container name"
    echo -e "  \e[32m--use_multi_container\e[0m      Enable multi-container mode"
    echo -e "  \e[32m--workspace <path>\e[0m         Path to mount into \$HOME/DoppelAutoware (default: pwd)"
    echo -e "  \e[32m--help\e[0m                     Show this help message"
    exit 0
}

# Parse named arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
    --container_name)
        CONTAINER_NAME="$2"
        shift 2
        ;;
    --use_multi_container)
        USE_MULTI_CONTAINER=true
        shift
        ;;
    --workspace)
        WORKSPACE_PATH="$2"
        shift 2
        ;;
    --help)
        show_help
        ;;
    *)
        echo -e "\e[31m[ERROR]: Unknown argument $1\e[0m"
        show_help
        ;;
    esac
done

# check if image is built
check_image() {
    if [[ "$(docker images -q $IMAGE_NAME 2>/dev/null)" == "" ]]; then
        echo -e "\e[31m[ERROR]: Image $IMAGE_NAME not found\e[0m"
        exit 1
    fi
}

if [[ "$USE_MULTI_CONTAINER" == true ]]; then
    IMAGE_NAME="${AUTOWARE_MULTI_IMAGE_NAME:-doppel_autoware_multi_container:latest}"
else
    IMAGE_NAME="${AUTOWARE_IMAGE_NAME:-doppel_autoware:latest}"
fi

check_image

if [[ "$WORKSPACE_PATH" == "" ]]; then
    WORKSPACE_PATH="$(pwd)"
fi

DATA_PATH="${AUTOWARE_DATA_HOST_PATH:-$WORKSPACE_PATH/data/autoware_data}"
if [[ ! -d "$DATA_PATH" ]]; then
    echo -e "\e[33m[WARN]: autoware_data directory not found at $DATA_PATH\e[0m"
    echo -e "\e[33m[WARN]: Set AUTOWARE_DATA_HOST_PATH to override the mount source path.\e[0m"
fi

# check if container is running
if [[ "$(docker ps -q -f name=$CONTAINER_NAME)" != "" ]]; then
    echo -e "\e[31m[ERROR]: Container $CONTAINER_NAME already running\e[0m"
    exit 1
fi

rocker --nvidia --privileged --x11 --user \
    --volume "$WORKSPACE_PATH":"$HOME/DoppelAutoware" \
    --volume "$DATA_PATH":"$HOME/autoware_data" \
    --name $CONTAINER_NAME \
    --mode non-interactive \
    --detach \
    -- $IMAGE_NAME sleep infinity

echo -e "\e[32m[INFO]: Started container $CONTAINER_NAME\e[0m"

# Ensure loopback multicast is enabled once per container start.
docker exec -u root "$CONTAINER_NAME" ip link set lo multicast on >/dev/null 2>&1 || true
