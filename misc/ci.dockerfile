FROM ros:jazzy AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Only packages with no rosdep key; everything else is declared in package.xml
# and installed below via rosdep.
RUN apt-get update && apt-get install --no-install-recommends -y \
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
    vim

COPY --from=ghcr.io/astral-sh/uv:0.8.18 /uv /uvx /bin/

RUN rosdep update

# ── Manifest extractor stage ────────────────────────────────────────────────────
# Pull the entire build context into a scratch stage, then copy out only the
# files needed for dependency installation.  This way the dep-install layer
# (below) is only invalidated when a package.xml / pyproject.toml / uv.lock
# actually changes — not on every source commit.
FROM base AS manifests
WORKDIR /src
COPY . .
RUN mkdir -p /out/src && \
    find src -name "package.xml" | while read f; do \
        mkdir -p "/out/$(dirname "$f")"; \
        cp "$f" "/out/$f"; \
    done && \
    cp Makefile mira.py pyproject.toml uv.lock /out/ && \
    cp -r misc /out/misc

# ── Builder stage ───────────────────────────────────────────────────────────────
FROM base AS builder
WORKDIR /workspace

# Install Python + ROS deps using only the manifests extracted above.
# Cached until a manifest changes; source commits do not invalidate this layer.
COPY --from=manifests /out/ .
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && make install-deps"

# Copy full source and build.  Only this layer re-runs on every commit.
COPY . /workspace
RUN /bin/bash -c "make build"
