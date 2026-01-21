# LARIS - Biomass UAV Stack

This repo aims to centralize all research work done by Biomass Group students from Laris (Laboratory of Autonomous Robots & Intelligent Systems) from UFSCar. 

- simulation-ws offers a centralized simulation env using MRS UAV System. You can create your own world/models and keep it there (see section Create new world)
- workspace creation: each student can create and manage your own workspace extended by simulation-ws. This way you can share your developed packages to work with an unified simulation repo (see Section create workspace).

Currently the repo was built entirely to use ROS Noetic. ROS 2 work is under progress.

## Setup

This repo contains submodules

``
git clone git@github.com:dederomagnolo/biomass-uav-stack.git
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

- Tools: `sudo apt install -y python3-catkin-tools python3-osrf-pycommon`

- Install [MRS System](https://github.com/ctu-mrs/mrs_uav_system)

### 2. Building

- From root run `catkin build`

- Set your build source: `source devel/setup.bash`

## Use Docker (Windows)

### 1. Docker setup

You can do your favorite config if you want. Suggestion:

- Install [Docker Desktop](https://www.docker.com/products/docker-desktop)

- [Configure your WSL](https://docs.docker.com/desktop/features/wsl)

### 2. Building image

First make sure the repo has been cloned under your WSL files. From repo root build `biomass-uav-stack` image. It take some time, go and grab a coffee.

``
docker build -t biomass-uav-stack .
``

### 3. Start your container

```
docker run -it --rm \
  --name biomass-uav-stack \
  --env DISPLAY="$DISPLAY" \
  --env WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  --env XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dxg \
  -v "$(pwd)/simulation-ws:/root/biomass-uav-ws" \
  biomass-uav-stack
```

### 4. Building

- You will land on `biomass-uav-ws` folder. This folder contains the content from `simulation-ws` repo folder.
- From root run `catkin build`
- Set your build source: `source devel/setup.bash`


## Run simulations

from root: `./tmux/one_drone/start.sh`

### Run simple launch file to test/edit

Set gazebo_resources path before run

`export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find gazebo_resources)/models`

`roslaunch mrs_uav_gazebo_simulation simulation.launch world_file:='$(find gazebo_resources)/worlds/cerrado_2.world'`

### Create new world

- worlds should be added to `gazebo_resources/worlds`

- models should be added to `gazebo_resources/models`

- functional template file with initial settings is available for insert models directly: `template.world`

## Troubleshooting

If your mavros is dying when gazebo is starting

`REQUIRED process [uav1/mavros-2] has died! │450 process has died [pid 4659, exit code -11, cmd`

install the packs below:

```
apt-get update
apt-get install -y geographiclib-tools
geographiclib-get-geoids egm96-5
```