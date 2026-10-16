# syntax=docker/dockerfile:1.7
# Dockerfile for Mira AUV Firmware
# Based on ROS2 Jazzy with Ubuntu 24.04 LTS
#
# Layers are ordered so a source edit only reruns the final colcon build:
# system/ROS packages depend on package.xml files, Python packages on
# pyproject.toml/uv.lock. apt and uv downloads are kept in BuildKit caches.

# package.xml manifests (and ignore markers) only, for rosdep.
FROM busybox:1.36 AS manifests
COPY src /src
RUN find /src -type f ! -name package.xml ! -name COLCON_IGNORE ! -name AMENT_IGNORE \
    ! -name CATKIN_IGNORE -delete

FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
# Keep downloaded .debs for the apt cache mount.
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
  && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

WORKDIR /workspace
COPY --from=manifests /src /workspace/src

# System, build, middleware and rosdep-resolved ROS dependencies in one apt
# transaction (the same set `make install-deps` installs on a host).
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rosdep update \
  && apt-get update \
  && rosdep_packages="$(rosdep install --from-paths src --ignore-src -r -s -y 2>/dev/null \
       | sed -n 's/.*apt-get install -y //p' | tr '\n' ' ')" \
  && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    curl \
    git \
    lld \
    ninja-build \
    pkg-config \
    ccache \
    python3-pip \
    usbutils \
    gstreamer1.0-plugins-base-apps \
    libgstreamer-plugins-good1.0-dev \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rtabmap-odom \
    ros-jazzy-rmw-zenoh-cpp \
    vim \
    $rosdep_packages

# Install uv (Python package manager)
COPY --from=ghcr.io/astral-sh/uv:0.8.18 /uv /uvx /bin/

# Python dependencies from the lock file. MIRA_EXTRAS=vision adds YOLO
# (ultralytics + PyTorch, several GB); the default image does not need it.
ARG MIRA_EXTRAS=""
ENV UV_LINK_MODE=copy
COPY pyproject.toml uv.lock /workspace/
RUN --mount=type=cache,target=/root/.cache/uv \
    uv venv --system-site-packages \
  && uv sync --frozen $(for extra in $MIRA_EXTRAS; do printf -- '--extra %s ' "$extra"; done)

# Optional native NVIDIA wheel. Supply an official URL matching Python/CUDA/arch.
# cuVSLAM links the CUDA math libraries that mira2_vision.cuvslam_odometry
# preloads (CUDA_LIBRARIES); install them explicitly rather than relying on
# PyTorch to bring them in.
ARG CUVSLAM_WHEEL_URL=""
RUN --mount=type=cache,target=/root/.cache/uv \
    if [ -n "$CUVSLAM_WHEEL_URL" ]; then uv pip install --python /workspace/.venv/bin/python "$CUVSLAM_WHEEL_URL" \
      nvidia-cublas-cu12 nvidia-cusolver-cu12 nvidia-cusparse-cu12 nvidia-nvjitlink-cu12; fi

# Source last: edits only invalidate the build below.
COPY . /workspace
RUN /bin/bash -c "make build"

# Entrypoint
RUN echo '#!/bin/bash\n\
source /opt/ros/jazzy/setup.bash\n\
source /workspace/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

EXPOSE 11311 14550

LABEL maintainer="Mira AUV Team" \
      description="Mira AUV Firmware - ROS2 Jazzy based autonomous underwater vehicle control system" \
      version="0.1.0"
