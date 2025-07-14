# docker/Dockerfile.cuda
FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04 as cuda

# Set up locale, tz, user, etc.
COPY --from=home_base / /  


# Add CUDA + OpenGL tools
USER root
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx libegl1-mesa libxext6 libx11-6 && \
    rm -rf /var/lib/apt/lists/*

################
# Expose the nvidia driver to allow opengl
# Dependencies for glvnd and X11.
################
RUN apt-get update \
 && apt-get install -y -qq --no-install-recommends \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6

# Create user
USER rosuser
WORKDIR /home/rosuser/ros2_ws
