# crazyflie_ros2_multiranger
This repository contains different ROS 2 nodes to interact with the multiranger on the Crazyflie for both simulation as the real Crazyflie.

## Installation

Start a workspace and clone the repository:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:knmcguire/crazyflie_ros2_multiranger.git
```

In the same workspace, git clone Crazyswarm2:
```bash

git clone https://github.com/IMRCLab/crazyswarm2 --recursive
```

There is a fix that needs to be merged so it's best to use the odom-tf-fix. For the rest make sure to [install all of Crazyswarm2's dependencies](https://imrclab.github.io/crazyswarm2/installation.html) but skip buildin

In the same workspace also install the ros_gz_crazyflie usage instructions in [the README of the ros_gz_crazyflie repository](https://github.com/knmcguire/ros_gz_crazyflie?tab=readme-ov-file#usage) for sourcing the specific Crazyflie model.
> Make sure to switch to the gazebo-multiranger branch when cloning the [crazyflie-simulation](https://github.com/bitcraze/crazyflie-simulation) repo!

Then build the workspace:
```bash
cd  ~/ros2_ws/
colcon build --cmake-args -DBUILD_TESTING=ON
source ~/ros2_ws/install/setup.bash
```

## Usage

Every terminal were you run the examples in needs to have the setup.bash sourced with:
```bash
source ~/ros2_ws/install/setup.bash
```

Also the simulation model needs to be sourced in every terminal where you run the simulation with:
```bash
export GZ_SIM_RESOURCE_PATH=/home/user/simulation_models/crazyflie_simulation/simulator_files/gazebo/"
```
> Note that the full directory should be sourced as tilde won't be recognized.

The latter is depended on where you have the simulation models stored.

> You might need to enter the full path for sourcing, so with `/home/USER/...` if you get a 'file not found' error.

### Simulated Crazyflie with simple mapper and teleop

To run the simulation with the simple mapper while controlling in with teleop run:

```bash
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py
```

In another terminal run teleop with
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Have the crazyflie take off by pressing 't' and follow the instructions of the teleop_twist_keyboard to control the Crazyflie. You can see the map being created in Rviz.

### Real Crazyflie with simple mapper and teleop

Go to crazyflie_ros2_multiranger_bringup/confg  and edit the crazyflie_real_crazyswarm2.yaml file to set the uri of the Crazyflie to the correct one.

Then run:
```bash
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_real.launch.py
```

In another terminal run teleop with
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Have the crazyflie take off by pressing 't' and follow the instructions of the teleop_twist_keyboard to control the Crazyflie. You can see the map being created in Rviz.

### Simulated Crazyflie with simple mapper and wall following

To run the simulation with the simple mapper while it is autonomously wall following run:

```bash
ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_simulation.launch.py
```

You don't have to control the  simulated Crazyflie as it is autonmously wallfollowing. You can see the map being created in Rviz.

You can make the simulated Crazyflie stop with calling the following service:
```bash
ros2 service call /crazyflie/stop_wall_following std_srvs/srv/Trigger
```

It will now stop moving and land.

### Real Crazyflie with simple mapper and wall following

Go to crazyflie_ros2_multiranger_bringup/config/  and edit the crazyflie_real_crazyswarm2.yaml file to set the uri of the Crazyflie to the correct one.

Then run:
```bash
ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_real.launch.py
```

You don't have to control the  simulated Crazyflie as it is autonomously wall following. You can see the map being created in Rviz.

You can make the simulated Crazyflie stop with calling the following service:
```bash
ros2 service call /crazyflie_real/stop_wall_following std_srvs/srv/Trigger
```

The crazyflie will now stop moving and land.

## New Features: SLAM and Localization Workflow (Simulation + Real)

This section documents the new bringup features added in this repository and how they connect into a full workflow:

1. Build a map with SLAM Toolbox.
2. Save the map to `.yaml` + `.pgm`.
3. Run AMCL localization on the saved map.
4. Send goals from RViz after localization is stable.

The structure follows the same staged navigation flow used in `Week-7-8-ROS2-Navigation`, but with Crazyflie-specific frames and topics.

### Prerequisites (every terminal)

Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Export simulation resources (adjust path for your machine):

```bash
export GZ_SIM_RESOURCE_PATH=/home/user/simulation_models/crazyflie_simulation/simulator_files/gazebo/
```

### 1. Mapping with SLAM Toolbox (Simulation)

Use the new SLAM mapping launch to start simulation + SLAM Toolbox + RViz together.

```bash
ros2 launch crazyflie_ros2_multiranger_bringup slamtb_mapper_simulation.launch.py world:=crazyflie_world.sdf
```

What this starts:
- Gazebo simulation (Crazyflie model)
- SLAM Toolbox in online async mode
- RViz with the SLAM mapping profile
- A static scan-frame TF bridge for multiranger compatibility

Expected behavior:
- The `/map` topic updates while the Crazyflie moves.
- The `map -> odom` transform is maintained by SLAM Toolbox.

### 2. Save the Generated Map

After exploring the environment, save the occupancy map:

```bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/cf_room2_map --ros-args -r map:=/map
```

This creates:
- `~/ros2_ws/maps/cf_room2_map.yaml`
- `~/ros2_ws/maps/cf_room2_map.pgm`

### 3. Localization on a Saved Map with AMCL (Simulation)

Run localization in two terminals.

Terminal 1: start simulation

```bash
source ~/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/user/simulation_models/crazyflie_simulation/simulator_files/gazebo/
ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py world:=crazyflie_world.sdf
```

Terminal 2: start AMCL localization on the saved map

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch crazyflie_ros2_multiranger_bringup localization_simulation.launch.py map:=/home/user/ros2_ws/maps/cf_room2_map.yaml
```

In RViz:
- Set `Fixed Frame` to `map`.
- Use `2D Pose Estimate` once to initialize AMCL.
- Verify map/laser/TF alignment.

How this is wired:
- `map_server` publishes the saved map.
- `amcl` estimates pose from multiranger scan + odometry.
- `map -> crazyflie/odom` is published by AMCL.

### 4. SLAM Mapping for Real Crazyflie

For real hardware, the SLAM launch now uses the local bringup SLAM config from this repository.

Before running, edit URI/settings in:
- `crazyflie_ros2_multiranger_bringup/config/crazyflie_real_crazyswarm2.yaml`

Then launch:

```bash
ros2 launch crazyflie_ros2_multiranger_bringup slamtb_mapper_real.launch.py
```

### 5. End-to-End Connection of the New Features

The new flow is designed to be used in sequence:

1. Run `slamtb_mapper_simulation.launch.py` to generate a map.
2. Save map with `map_saver_cli`.
3. Run `localization_simulation.launch.py` with the saved `.yaml` map.
4. Set initial pose in RViz and then send goals.

This gives the same staged logic as the Week-7-8 tutorial (`mapping -> saved map -> localization -> goal-driven operation`) while keeping the Crazyflie-specific setup and topic/frame conventions in `ros2_ws`.
