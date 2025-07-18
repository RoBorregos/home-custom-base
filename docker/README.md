## Getting Started

Before using the Docker containers in this project, make sure Docker is properly installed on your system. Refer to the [official Docker documentation](https://docs.docker.com/engine/install/) to install the Docker Engine.

If your system supports GPU acceleration, you’ll also need to install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) and the appropriate GPU drivers. This enables CUDA integration and GPU support within Docker containers.

---

## Project Structure

This repository uses the following directory structure to organize Docker-related resources:
```
docker/
├── compose
│   └── docker-compose.yml
├── doc
│   └── Docker.rst
├── dockerfiles
│   ├── nav.dockerfile
│   └── ros.dockerfile
├── README.md
└── scripts
    └── home_base.sh
```

---

## Docker Deployment

Docker integration in this repository is handled through the `home_base.sh` script. This script provides a simple interface for building images, deploying containers, and entering development environments using `docker-compose`.

### Base Images

This setup defines two base images:

- **`roborregos/home_base:cpu_base`**  
  A lightweight Ubuntu 22.04 image with minimal ROS 2 installed, designed for systems without GPU support.

- **`roborregos/home_base:cuda_base`**  
  Based on `nvidia/cuda-runtime-11.8`, this image includes ROS 2 and supports full CUDA development capabilities.

These images are defined in the `docker-compose.yml` file and are used as the foundation for all container builds.

### CPU vs CUDA Base Images

Both base images provide ROS 2 development support, but they differ in GPU capabilities:

- **CPU Base Image (`cpu_base`)**  
  Designed for machines without a CUDA-compatible GPU or NVIDIA Container Toolkit. It’s built on Ubuntu 22.04 docker image.  
  If your system has NVIDIA drivers and the container toolkit installed, this image *can still access the GPU* using the `--gpu` flag for GPU-accelerated rendering (e.g., Gazebo, OpenGL).  
  This makes it ideal for lightweight setups that don’t require the full CUDA toolchain.  
  *Note: Support for non-NVIDIA GPUs is currently unavailable.*

- **CUDA Base Image (`cuda_base`)**  
  Designed for full CUDA development. This image supports GPU compute libraries and tools for advanced GPU workloads.  
  Use this if your development depends on CUDA features, such as neural network inference or GPU-based SLAM.

## Nav Image

Depending on the user input, the appropriate base image is selected and used to build the corresponding `nav_[--flag]` image. This Nav image is constructed from a separate Dockerfile called `nav.dockerfile`, which installs the necessary dependencies for developing with SLAM Toolbox and Navigation2.

> **Note:** The `nav.dockerfile` can be modified to suit specific user requirements. Additionally, custom Dockerfiles can be added and referenced in `docker-compose.yml` to build containers with alternative or extended ROS 2 package dependencies. The `ros.dockerfile` provides minimal ROS 2 support for the base images as a foundation for these customizations.

---

## Deploy Guide

The `home_base.sh` script simplifies the Docker container deployment workflow into three main instructions:

```bash
# Builds the base image and the corresponding nav image based on the selected flag.
# Then deploys the Docker container and provides CLI access.
./scripts/home_base.sh -deploy [--deploy-flag]

# Stops one or multiple containers defined in the docker-compose file.
./scripts/home_base.sh -stop [--stop-flag]

# Removes one or multiple deployed containers.
./scripts/home_base.sh -remove [--remove-flag]
```

### Flags

#### Deploy Flags

- `--cpu`  
  Builds a minimal Docker container based on `ubuntu:22.04` with CPU rendering support. Includes ROS 2, Navigation2 (`Nav2`), SLAM Toolbox, and basic GUI tools.

- `--gpu`  
  Similar to `--cpu`, but enables NVIDIA GPU acceleration using the NVIDIA Container Toolkit. Suitable for machines with GPU hardware.

- `--cuda`  
  Uses `nvidia/cuda-runtime:11.8` as the base image. Provides full CUDA support for leveraging host GPU capabilities in compute-intensive workloads.


#### Stop and Remove Flags

- `--[base image]`  
  Specifies which container to stop or remove. Valid values include `--cpu`, `--gpu`, or `--cuda`, matching the deployed base image.

- `--all`  
  Applies the stop or remove action to **all containers** defined in the `docker-compose.yml` file.

---

### Supported Commands

You can run the script using different flags depending on your needs:

```bash
./scripts/home_base.sh [COMMAND] [--gpu|--cuda]
```
#### Examples
```bash
# Build, deploy, and enter a CPU-based container (default)
./scripts/home_base.sh -deploy

# Same as above, explicitly specifying CPU
./scripts/home_base.sh -deploy --cpu

# Deploy with GPU support for OpenGL or Gazebo rendering
./scripts/home_base.sh -deploy --gpu

# Deploy with full CUDA development stack
./scripts/home_base.sh -deploy --cuda

# Stop all running containers
./scripts/home_base.sh -stop all

# Remove all containers and clean up resources
./scripts/home_base.sh -remove all

# Build only base image (CPU or CUDA depending on flag)
./scripts/home_base.sh -build-base --cuda

# Build only the Nav2 image with GPU support
./scripts/home_base.sh -build-nav2 --gpu

```
