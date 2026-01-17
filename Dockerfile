# ========================================
#  Base: Ubuntu 20.04 (for ROS Noetic)
# ========================================
FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ========================================
#  Basic dependencies
# ========================================
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release sudo git \
    build-essential cmake python3-pip nano dos2unix

# ========================================
#  ROS Noetic Repo & Installation
# ========================================
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y ros-noetic-desktop-full

# ========================================
#  ROS Additional Tools 
# ========================================
RUN apt-get install -y python3-catkin-tools python3-rosdep && \
    rosdep init && rosdep update

# ========================================
# Gazebo + ROS integration
# ========================================
RUN apt-get update && apt-get install -y \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control

# ========================================
# Misc Dependencies
# ========================================
RUN apt-get update && apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-msgs \
    ros-noetic-mavros-extras \
    geographiclib-tools

RUN apt-get update && sudo apt install -y \
    ros-noetic-rosbridge-server \
    ros-noetic-random-numbers

# ========================================
#  MRS System
# ========================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://ctu-mrs.github.io/ppa-stable/add_ppa.sh | bash && \
    apt-get update && \
    apt-get install -y ros-noetic-mrs-uav-system-full && \
    rm -rf /var/lib/apt/lists/*

# ========================================
#  Env configs
# ========================================
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    mkdir -p /root/biomass-uav-ws/src && \
    echo "source ~/biomass-uav-ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /root/biomass-uav-ws

CMD ["bash"]
