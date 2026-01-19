# ========================================
#  Base: Ubuntu 24.04 (for ROS 2 Jazzy)
# ========================================
FROM ubuntu:24.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ========================================
#  Basic dependencies
# ========================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl gnupg2 lsb-release sudo git \
    && rm -rf /var/lib/apt/lists/*

# ========================================
#  ROS 2 Jazzy Installation
# ========================================
RUN curl -fsSL https://ctu-mrs.github.io/ppa2-stable/add_ros_ppa.sh | bash && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-jazzy-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# ========================================
#  ROS 2 build / tools
# ========================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# rosdep init/update
RUN rosdep init && rosdep update

# ========================================
#  MRS System (ROS 2)
# ========================================
RUN curl -fsSL https://ctu-mrs.github.io/ppa2-stable/add_ppa.sh | bash && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-jazzy-mrs-uav-system-full \
    && rm -rf /var/lib/apt/lists/*

# ========================================
#  Zenoh RMW (set as default)
# ========================================
# Binaries are available as ros-jazzy-rmw-zenoh-cpp
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# ========================================
#  Env configs + workspace skeleton
# ========================================
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo 'export RMW_IMPLEMENTATION="rmw_zenoh_cpp"' >> /root/.bashrc && \
    mkdir -p /root/biomass-uav-ws/src && \
    echo '[ -f /root/biomass-uav-ws/install/setup.bash ] && source /root/biomass-uav-ws/install/setup.bash' >> /root/.bashrc

WORKDIR /root/biomass-uav-ws
CMD ["bash"]
