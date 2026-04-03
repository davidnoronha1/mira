# Dockerfile for Mira AUV Firmware
# Based on ROS2 Jazzy with Ubuntu 24.04 LTS
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system + build + runtime dependencies in one layer
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    curl \
    git \
    lld \
    ninja-build \
    pkg-config \
    python3-pip \
    usbutils \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-plugins-base-apps \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-tools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libgstrtspserver-1.0-dev \
    ros-jazzy-behaviortree-cpp \
    libboost-python1.74-dev \
    vim

# Install uv (Python package manager)
COPY --from=ghcr.io/astral-sh/uv:0.8.18 /uv /uvx /bin/

# Initialize rosdep
RUN rosdep update

WORKDIR /workspace

# Copy project files
# RUN git clone https://github.com/Dreadnought-Robotics/mira /workspace
COPY . /workspace

# Install Python dependencies using uv
# RUN uv sync

# Install ROS dependencies
# Installs uv dependencies
# Installs a few build dependencies
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && make install-deps"

# Clean previous build artifacts
RUN rm -rf ./build ./log ./install /var/lib/apt/lists/*

# Build the workspace
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
