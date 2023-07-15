# Docker hub user name
ARG DOCKER_HUB_USER_NAME

# Use pylon
FROM $DOCKER_HUB_USER_NAME/ros-pylon:7.3.0 AS pylon

# Base image
FROM ros:humble

# Global args
# Add a non-root user to a container
# For more information, visit: https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash --create-home $USERNAME

# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Enable root user access ros
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Copy from pylon
COPY --from=pylon /setup /

# Setup environment
ENV PYLON_ROOT=/opt/pylon

# Install dependencies
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    gdb \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Setup ldconfig
RUN ldconfig

# Login as ros
USER ros
