#!/bin/bash

DEFAULT_BASE_TAG="cpu_base"
CUDA_BASE_TAG="cuda_base"
GPU_TAG="nav2_gpu"
BASE_REPO="roborregos/home_base"
NAV2_REPO="roborregos/nav2"

# Util
function parse_gpu_flag() {
    local use_gpu=false
    for arg in "$@"; do
        if [[ "$arg" == "--gpu" ]]; then
            use_gpu=true
        fi
    done
    echo "$use_gpu"
}

function get_base_tag() {
    local gpu_flag
    gpu_flag=$(parse_gpu_flag "$@")
    if [[ "$gpu_flag" == "true" ]]; then
        # Temprarily disable CUDA support
        # echo "$CUDA_BASE_TAG"
        echo "$DEFAULT_BASE_TAG"
    else
        echo "$DEFAULT_BASE_TAG"
    fi
}

function get_nav2_service() {
    local gpu_flag
    gpu_flag=$(parse_gpu_flag "$@")
    if [[ "$gpu_flag" == "true" ]]; then
        echo "nav2_gpu"
    else
        echo "nav2_cpu"
    fi
}

# Step-by-step helpers
function build_base_image() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    echo "🚧 Building base image: $base_tag"
    docker compose build "$base_tag"
}

function build_nav2_image() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    local service
    service=$(get_nav2_service "$@")

    echo "🔧 Building Nav2 image: $service (based on $base_tag)"
    NAV2_BASE_IMAGE="$BASE_REPO:$base_tag" \
    NAV2_BASE_IMAGE_TAG="$base_tag" \
    docker compose build "$service"
}

function run_container() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    local base_image="$BASE_REPO:$base_tag"
    local service
    service=$(get_nav2_service "$@")

    echo "🚀 Starting container: $service"
    xhost +local:docker
    NAV2_BASE_IMAGE="$base_image" \
    NAV2_BASE_IMAGE_TAG="$base_tag" \
    docker compose up -d "$service"
}

function attach_shell() {
    local service
    service=$(get_nav2_service "$@")

    echo "🧑‍💻 Attaching to $service shell..."
    docker exec -it "$service" bash
}

# Top-level operations
function deploy() {
    build_base_image "$@"
    build_nav2_image "$@"
    run_container "$@"
    attach_shell "$@"
}

function dev_mode() {
    run_container "$@"
    attach_shell "$@"
}

function stop_container() {
    local service
    service=$(get_nav2_service "$@")

    echo "🛑 Stopping container: $service"
    docker compose stop "$service"
}

function remove_all() {
    echo "🧹 Removing all containers and resources..."
    docker compose down
}

function help_message() {
    echo "Usage: ./home_base.sh [COMMAND] [--gpu]"
    echo
    echo "Commands:"
    echo "  -build-base [--gpu]      Build only the base image"
    echo "  -build-nav2 [--gpu]      Build only the Nav2 image"
    echo "  -run [--gpu]             Start only the container"
    echo "  -dev-mode [--gpu]        Run + attach to shell"
    echo "  -deploy [--gp]          Build everything, run, and attach"
    echo "  -remove                  Remove all containers"
    echo "  -help                    Show this help message"
    echo
    echo "Examples:"
    echo "  ./home_base.sh -deploy"
    echo "  ./home_base.sh -dev-mode --gpu"
    echo "  ./home_base.sh -build-base"
}

# Main dispatcher
case "$1" in
    -build-base)
        build_base_image "$@"
        ;;
    -build-nav2)
        build_nav2_image "$@"
        ;;
    -run)
        run_container "$@"
        ;;
    -dev-mode)
        dev_mode "$@"
        ;;
    -deploy)
        deploy "$@"
        ;;
    -remove)
        remove_all
        ;;
    -help|--help)
        help_message
        ;;
    -stop)
        stop_container "$@"
        ;;
    *)
        echo "Unknown command: $1"
        help_message
        exit 1
        ;;
esac
