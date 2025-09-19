# Dockerfile for Mira AUV Firmware
# Based on ROS2 Jazzy with Ubuntu 24.04 LTS
FROM ros:jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    build-essential \
    cmake \
    pkg-config \
    libusb-1.0-0-dev \
    libuvc-dev \
    python3-pip

# Install uv (Python package manager)
COPY --from=ghcr.io/astral-sh/uv:0.8.18 /uv /uvx /bin/

# Initialize rosdep
RUN rosdep update

# Set working directory
WORKDIR /workspace

# Copy project files
COPY . .

# Install Python dependencies using uv
RUN uv sync

# Install ROS dependencies
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    make install-deps"

# RUN rm -rf /var/lib/apt/lists/*

RUN rm -rf ./build ./log ./install ./.venv

# RUN apt install libusb-1.0-0-dev -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    source .venv/bin/activate && \
    make build"

# Create a script to source the environment
RUN echo '#!/bin/bash\n\
source /opt/ros/jazzy/setup.bash\n\
source .venv/bin/activate\n\
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