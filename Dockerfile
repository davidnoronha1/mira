# Dockerfile for Mira AUV Firmware
# Based on ROS2 Jazzy with Ubuntu 24.04 LTS
FROM ros:jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    curl \
    build-essential \
    cmake \
    pkg-config \
    lld \
    ninja-build \
    python3-pip \
    usbutils

# Install uv (Python package manager)
COPY --from=ghcr.io/astral-sh/uv:0.8.18 /uv /uvx /bin/

# Initialize rosdep
RUN rosdep update

# Set working directory
WORKDIR /workspace

# Copy project files
RUN git clone https://github.com/Dreadnought-Robotics/mira /workspace

# Install Python dependencies using uv
RUN uv sync

# Install ROS dependencies
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    make install-deps"

# RUN rm -rf /var/lib/apt/lists/*

RUN rm -rf ./build ./log ./install

# RUN apt install libusb-1.0-0-dev -y

RUN apt-get install --no-install-recommends -y \
    usbutils \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-plugins-base-apps \
    libgstreamer1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-tools \
    vim

# Build the workspace
RUN /bin/bash -c "make build"

# Create a script to source the environment
RUN echo '#!/bin/bash\n\
source /opt/ros/jazzy/setup.bash\n\
source install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]

# Expose common ROS ports
EXPOSE 11311 14550

# Set labels
LABEL maintainer="Mira AUV Team"
LABEL description="Mira AUV Firmware - ROS2 Jazzy based autonomous underwater vehicle control system"
LABEL version="0.1.0"
