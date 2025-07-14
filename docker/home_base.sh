#!/bin/bash

# home_base.sh - Simple launcher for the Docker development environment

DEFAULT_SERVICE="home_base"

function build_image() {
    local service="${1:-$DEFAULT_SERVICE}"
    echo "🚧 Building Docker image for service: $service"
    USER_UID=$(id -u)
    USER_GID=$(id -g)

    docker compose build --build-arg USER_UID=$USER_UID --build-arg USER_GID=$USER_GID "$service"
}

function start_container() {
    local service="${1:-$DEFAULT_SERVICE}"
    echo "🚀 Starting container for service: $service"
    xhost +local:docker  # Allow container to use X11 for GUI apps
    docker compose up -d "$service"
}

function dev_mode() {
    local container="${1:-$DEFAULT_SERVICE}"
    echo "🧑‍💻 Entering development mode for container: $container"
    docker exec -it "$container" bash
}

function stop_container() {
    local service="${1:-$DEFAULT_SERVICE}"
    echo "🛑 Stopping container for service: $service"
    docker compose stop "$service"
}

function remove_container() {
    local service="${1:-$DEFAULT_SERVICE}"
    echo "🗑️ Removing container and resources for service: $service"
    docker compose down
}

function help_message() {
    echo "Usage: ./home_base.sh [OPTION] [SERVICE_NAME]"
    echo
    echo "Options:"
    echo "  -build [SERVICE]     Build the Docker image for a service (default: home_base)"
    echo "  -run [SERVICE]       Start the container (default: home_base)"
    echo "  -dev-mode [CONTAINER] Attach to container terminal (default: home_base)"
    echo "  -stop [SERVICE]      Stop the container (default: home_base)"
    echo "  -remove [SERVICE]    Remove the container and its resources (default: home_base)"
    echo "  -help                Show this help message"
    echo
}

# Parse arguments
case "$1" in
    -build)
        build_image "$2"
        ;;
    -run)
        start_container "$2"
        ;;
    -dev-mode)
        dev_mode "$2"
        ;;
    -stop)
        stop_container "$2"
        ;;
    -remove)
        remove_container "$2"
        ;;
    -help|*)
        help_message
        ;;
esac
