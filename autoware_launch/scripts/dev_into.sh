# !/bin/bash

CONTAINER_NAME="autoware_dev_${USER}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-}"
START_RECEIVER=false

derive_ros_domain_id() {
    local container_name="$1"
    if [[ "$container_name" =~ ([0-9]+)$ ]]; then
        echo "${BASH_REMATCH[1]}"
        return 0
    fi
    return 1
}

show_help() {
    echo -e "\e[34mUsage:\e[0m $0 [options]"
    echo -e "\e[34mOptions:\e[0m"
    echo -e "  \e[32m--container_name <name>\e[0m    Attach to container name"
    echo -e "  \e[32m--ros-domain-id <id>\e[0m      Export ROS_DOMAIN_ID for the attached shell"
    echo -e "  \e[32m--start-receiver\e[0m         Start autoware_controller/receiver.py after attach"
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
    --ros-domain-id)
        ROS_DOMAIN_ID_VALUE="$2"
        shift 2
        ;;
    --start-receiver)
        START_RECEIVER=true
        shift
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

# check if container is running
check_container() {
    if [[ "$(docker ps -q -f name=$CONTAINER_NAME)" == "" ]]; then
        echo -e "\e[31m[ERROR]: Container $CONTAINER_NAME not running\e[0m"
        exit 1
    fi
}

check_container

if [[ -z "$ROS_DOMAIN_ID_VALUE" ]]; then
    ROS_DOMAIN_ID_VALUE="$(derive_ros_domain_id "$CONTAINER_NAME" || true)"
fi

echo -e "\e[32m[INFO]: Attaching to container $CONTAINER_NAME\e[0m"
if [[ -n "$ROS_DOMAIN_ID_VALUE" ]]; then
    echo -e "\e[32m[INFO]: ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE for attached shell\e[0m"
fi

DOCKER_EXEC_ARGS=(
  -it
  -e DISPLAY=$DISPLAY
  -e QT_X11_NO_MITSHM=1
)
if [[ -n "$ROS_DOMAIN_ID_VALUE" ]]; then
    DOCKER_EXEC_ARGS+=(-e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE")
fi

SHELL_COMMAND='source /ros_entrypoint.sh'
if [[ "$START_RECEIVER" == true ]]; then
  echo -e "\e[32m[INFO]: Starting receiver automatically in $CONTAINER_NAME\e[0m"
  SHELL_COMMAND+=' && cd "$HOME/DoppelAutoware" && python3 autoware_controller/receiver.py; exec bash'
else
  SHELL_COMMAND+=' && exec bash'
fi

docker exec "${DOCKER_EXEC_ARGS[@]}" \
  $CONTAINER_NAME \
  bash -lc "$SHELL_COMMAND"
