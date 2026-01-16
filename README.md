# LARIS - Biomass UAV Stack

## Setup

This repo contains submodules

``
git clone --recurse-submodules git@github.com:dederomagnolo/biomass-uav-stack.git
``

## Build from Source - Ubuntu 20.04

### 1. Dependencies

- Install ROS Noetic

```bash
curl https://ctu-mrs.github.io/ppa-stable/add_ros_ppa.sh | bash
sudo apt install ros-noetic-desktop-full
```

Obs: don't forget to add ros source to your terminal

Per terminal:
``source /opt/ros/noetic/setup.bash``

To do it only once on your system:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

- Install [MRS System](https://github.com/ctu-mrs/mrs_uav_system)

### 2. Building

- From root run `catkin build`

- Set your build source: `source devel/setup.bash`

### 3. Test Installation

`roslaunch gazebo_utils full_sim.launch`

### 4. Custom worlds

World files should be placed on: `mrsl_quadrotor_description/worlds`

Models should be placed on: `mrsl_models/models`

Edit `full_sim.launch` to point to your world name at tag:

``
<arg name="world_model" default="example-world" />
``

## Use Docker (Windows)

1. Setup your Docker Desktop and Use WSL

2. From root build `biomass-uav-stack` image. It take some time, go and grab a coffee.

``
docker build -t biomass-uav-stack .
``

3. Start your container

```
docker run -it --rm \
  --name biomass-uav-stack \
  --env DISPLAY="$DISPLAY" \
  --env WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  --env XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dxg \
  -v "$(pwd)/workspace:/root/biomass-uav-ws" \
  biomass-uav-stack
```

4. You will land on `biomass-uav-ws` folder. This folder contains the content from `workspace` repo folder.

5. Building

### 2. Building

- From root run `catkin build`

- Set your build source: `source devel/setup.bash`

### 3. Test Installation

`roslaunch gazebo_utils full_sim.launch`
