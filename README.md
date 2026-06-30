# LARIS - Biomass UAV Stack

This repo aims to centralize workspaces used by Biomass Group students from Laris (Laboratory of Autonomous Robots & Intelligent Systems) from UFSCar. The idea is to turn easy experimento reproduction and configuration sharing. 

- simulation-ws offers a centralized simulation env using MRS UAV System. The simulation worlds and models used by this stack are based on the `biomass-simulation-resources` package, which centralizes reusable Gazebo assets such as `worlds`, `models`, `ground_truth`, and helper scripts for forest scenarios.
- workspace creation: each student can create and manage your own workspace extended by simulation-ws. This way you can share your developed packages to work with an unified simulation repo (see Section create workspace).

# Setup

## Build from Source - Ubuntu 20.04

### 1. Dependencies

- Install ROS Noetic

```bash
curl https://ctu-mrs.github.io/ppa-stable/add_ros_ppa.sh | bash
sudo apt install ros-noetic-desktop-full
```

Obs: don't forget to add ros source to your terminal

Per terminal: `source /opt/ros/noetic/setup.bash`

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

## Use Docker (ROS 1 / Noetic on Windows)

### 1. Docker setup

You can do your favorite config if you want. Suggestion:

- Install [Docker Desktop](https://www.docker.com/products/docker-desktop)

- [Configure your WSL](https://docs.docker.com/desktop/features/wsl)

### 2. Building image

First make sure the repo has been cloned under your WSL files. From repo root build the `biomass-uav-stack` image. It takes some time, go and grab a coffee.

```
docker build -t biomass-uav-stack .
```

### 3. Start your container

Use Docker Compose from the repo root:

```
docker compose up -d --build
```

Enter the container with:

```
docker exec -it biomass-uav-stack bash
```

To stop it:

```
docker compose down
```

The GPU, WSLg mounts, display variables, and workspace bind are already defined in [compose.yaml](./compose.yaml). By default, it mounts `tree-ws` into `/root/biomass-uav-ws`.

### 4. Building

- You will land on `biomass-uav-ws` folder. With the default `compose.yaml`, this folder is a bind mount of `tree-ws`.
- From root run `catkin build`
- Set your build source: `source devel/setup.bash`

## Use Docker (ROS 2 / Jazzy on Windows)

The ROS 2 container definition lives in [`ros2-docker/Dockerfile`](./ros2-docker/Dockerfile). A matching Compose file is available at [`compose.ros2.yaml`](./compose.ros2.yaml).

### 1. Start the ROS 2 container

From the repo root:

```bash
docker compose -f compose.ros2.yaml up -d --build
```

Enter the container with:

```bash
docker exec -it biomass-uav-stack-ros2 bash
```

Stop it with:

```bash
docker compose -f compose.ros2.yaml down
```

By default, [`compose.ros2.yaml`](./compose.ros2.yaml) mounts `ros2-ws` into `/root/biomass-uav-ws`.

### 2. Build a ROS 2 workspace

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd /root/biomass-uav-ws
colcon build
source install/setup.bash
```

## Run simulations

from root: `./tmux/one_drone/start.sh`

### Run simple launch file to test/edit

Set gazebo_resources path before run

`export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find gazebo_resources)/models`

`roslaunch mrs_uav_gazebo_simulation simulation.launch world_file:='$(find gazebo_resources)/worlds/cerrado_2.world'`

### Create new world

- The simulation worlds used in this repository are based on the `biomass-simulation-resources` package.
- Briefly, `biomass-simulation-resources` is the package that stores reusable Gazebo assets for our forest experiments: world files, object models, ground-truth data, and support utilities for visualization and generation.
- In this repository, you can find it under:
  - `tree-ws/src/biomass-simulation-resources`
  - `simulation-ws/src/biomass-simulation-resources`
- To create a new world, use these files as the main reference:
  - [`tree-ws/src/biomass-simulation-resources/worlds/world_template.world`](./tree-ws/src/biomass-simulation-resources/worlds/world_template.world)
  - [`simulation-ws/scripts/generate_forest.py`](./simulation-ws/scripts/generate_forest.py)
- As a rule:
  - new `.world` files should be added to `biomass-simulation-resources/worlds`
  - new Gazebo models should be added to `biomass-simulation-resources/models`
  - if the world has reference tree positions, keep the corresponding files in `biomass-simulation-resources/ground_truth`

- The recommended path is to follow this README section together with the template world above. If you want a procedural example for randomized forests, use `simulation-ws/scripts/generate_forest.py` as the reference implementation.

## Troubleshooting

If your mavros is dying when gazebo is starting

`REQUIRED process [uav1/mavros-2] has died! │450 process has died [pid 4659, exit code -11, cmd`

install the packs below:

```
apt-get update
apt-get install -y geographiclib-tools
geographiclib-get-geoids egm96-5
```
