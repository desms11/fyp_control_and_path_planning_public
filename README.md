# FYP UAV Control and Path Planning

This repository is a public snapshot of a final-year project on Crazyflie path planning and control. It combines ROS 2/Crazyswarm2 flight workflows, simulator assets, trajectory scripts, navigation benchmark outputs, and post-run analysis artifacts.

The project is useful from two angles:

- For academic readers, it documents a benchmarked comparison of path-planning approaches and the supporting simulation/test pipeline.
- For robotics developers, it provides a working Crazyflie/Crazyswarm2 workspace with map generation, trajectory execution, rosbag conversion, and report generation scripts.

## Acknowledgments

This project builds upon and integrates the following open-source repositories:

- **ros_gz_crazyflie** – Gazebo simulation and ROS bridge for Crazyflie
- **crazyflie_ros2_multiranger** – ROS 2 integration with multiranger sensor
- **crazyswarm2** – ROS 2 Crazyflie swarm framework
- **crazyflie-simulation** – Simulator assets and documentation

## What Is In This Repo

The top-level repository is mostly a public wrapper around the embedded workspace in `public_fyp_repo/`. The main contents are:

- `public_fyp_repo/src/crazyswarm2/` – ROS 2/Crazyswarm2 workspace with Crazyflie examples, system tests, plotting utilities, and trajectory CSVs.
- `public_fyp_repo/src/crazyflie-simulation/` – Simulator assets and documentation for Crazyflie models in Webots and Gazebo.
- `public_fyp_repo/src/ros_gz_crazyflie/` – Gazebo-based Crazyflie integration and control assets.
- `public_fyp_repo/src/crazyflie_ros2_multiranger/` – Navigation and multiranger integration.
- `public_fyp_repo/maps/` and `public_fyp_repo/reports/` – Generated maps, benchmark reports, and analysis outputs.
- `data/`, `models/`, `notebooks/`, and `results/` at the top level – Project artifacts used for analysis and reporting.

## Prerequisites

**Recommended environment:** Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic

Ensure you have:
- Git
- Python 3
- Build essentials (colcon, rosdep)

## Installation

### 1. Install ROS 2 Jazzy and Gazebo Harmonic

Follow the official installation guides for [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) and [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install).

### 2. Install Basic Tools

```bash
sudo apt update
sudo apt install -y git python3-rosdep python3-colcon-common-extensions
```

### 3. Initialize rosdep

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

### 4. Set Gazebo Model Resource Path

Set `GZ_SIM_RESOURCE_PATH` so Gazebo can resolve the Crazyflie model:

```bash
export GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/crazyflie-simulation/simulator_files/gazebo
```

To persist this in your shell:

```bash
echo 'export GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/crazyflie-simulation/simulator_files/gazebo' >> ~/.bashrc
source ~/.bashrc
```

### 5. Install Dependencies

From the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 6. Build the Workspace

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
source ~/ros2_ws/install/setup.bash
```

## Pathfinding Algorithms

This workspace supports four pathfinding algorithms for point-to-point navigation:

| Algorithm | Type | Description |
|-----------|------|-------------|
| **Dijkstra** | Deterministic, optimal | Exhaustive grid search; guarantees shortest path |
| **A*** | Deterministic, heuristic | Grid search with Euclidean heuristic; same optimal path, faster planning |
| **RRT** | Sampling-based, probabilistic | Rapidly-exploring Random Tree; explores free space via random sampling |
| **GA** | Evolutionary, stochastic | Genetic Algorithm; evolves candidate paths over generations |

**Implementation note:** Dijkstra and A* use Nav2's built-in `NavfnPlanner` and execute via `NavigateToPose` action. RRT and GA are custom Python implementations in `scripts/planners.py` that plan on the occupancy grid and execute via Nav2's `FollowPath` action.

## Usage

### Quick Start: Navigation Simulation

#### Terminal 1 – Start Navigation Simulation

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models:~/ros2_ws/src/crazyflie-simulation/simulator_files/gazebo
ros2 launch crazyflie_ros2_multiranger_bringup navigation_simulation.launch.py
```

#### Terminal 2 – Navigate with Your Chosen Algorithm

Source your environment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Then run one of the following commands:

**Dijkstra (default):**

```bash
ros2 run crazyflie_ros2_multiranger_bringup navigate_to_pose.py \
    --ros-args -p use_sim_time:=true \
    -p goal_x:=3.0 -p goal_y:=2.0 -p goal_yaw:=0.0 \
    -p planner:=Dijkstra
```

**A*:**

```bash
ros2 run crazyflie_ros2_multiranger_bringup navigate_to_pose.py \
    --ros-args -p use_sim_time:=true \
    -p goal_x:=3.0 -p goal_y:=2.0 -p goal_yaw:=0.0 \
    -p planner:=AStar
```

**RRT:**

```bash
ros2 run crazyflie_ros2_multiranger_bringup navigate_to_pose.py \
    --ros-args -p use_sim_time:=true \
    -p goal_x:=3.0 -p goal_y:=2.0 -p goal_yaw:=0.0 \
    -p planner:=RRT
```

**GA:**

```bash
ros2 run crazyflie_ros2_multiranger_bringup navigate_to_pose.py \
    --ros-args -p use_sim_time:=true \
    -p goal_x:=3.0 -p goal_y:=2.0 -p goal_yaw:=0.0 \
    -p planner:=GA
```

Each run automatically generates a JSON performance report in `~/ros2_ws/reports/`.

### Custom Drone Spawn Coordinates

The navigation launch file supports spawning the drone at arbitrary map coordinates:

```bash
ros2 launch crazyflie_ros2_multiranger_bringup navigation_simulation.launch.py \
    spawn_x:=1.0 spawn_y:=2.0 spawn_yaw:=1.57
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `spawn_x` | 0.0 | Initial X position on map (meters) |
| `spawn_y` | 0.0 | Initial Y position on map (meters) |
| `spawn_yaw` | 0.0 | Initial heading (radians) |

**Note:** The launch file handles teleportation (at t=5s) and AMCL seeding (at t=12s) automatically. The map and crazyflie/odom TF frames remain at (0,0); AMCL computes the transform offset to represent the drone's actual position.

### Finding Valid Goal Coordinates

To visualize the map and find valid goal coordinates:

```bash
python3 ~/ros2_ws/scripts/show_map_coords.py
```

Click on white (free) areas of the map to see world coordinates. To use a different map:

```bash
python3 ~/ros2_ws/scripts/show_map_coords.py /path/to/your_map.yaml
```

## Benchmarking and Analysis

### Performance Report Format

Each navigation run generates a JSON file in `~/ros2_ws/reports/` with these key performance indicators (KPIs):

| Metric | Key | Description |
|--------|-----|-------------|
| Path Length (planned) | `path_length_planned_m` | Length of the initial planned path in meters |
| Distance Traveled | `distance_traveled_m` | Actual odometry distance the drone flew |
| Planning Time | `planning_time_s` | Wall-clock time to compute the path |
| Execution Time | `execution_time_s` | Time from goal sent to goal reached |
| Total Time | `total_time_s` | Planning + execution time |
| Completeness | `completeness` | 1 if goal reached, 0 otherwise |
| Energy Efficiency | `energy_efficiency` | Ratio of planned path length to actual distance |
| Path Smoothness | `path_smoothness_rad` | Average angular change along the planned path |
| Recoveries | `num_recoveries` | Number of Nav2 recovery behaviors triggered |
| Avg Velocity | `avg_velocity_ms` | Mean speed during execution (m/s) |

### GUI Benchmark Analyzer

A graphical tool for interactive analysis:

```bash
python3 ~/ros2_ws/src/crazyflie_ros2_multiranger/crazyflie_ros2_multiranger_bringup/scripts/compare_reports_gui.py
```

**Features:**

- Sortable table of all runs (click column headers)
- Filter by algorithm, map name, and status
- Algorithm summary panel (mean ± std)
- **File > Export CSV** to save filtered data
- **Analysis > Run Statistics:** Shapiro-Wilk normality test, Kruskal-Wallis H-test, Mann-Whitney U pairwise post-hoc tests
- **Analysis > Generate Plots:** Box plots for key metrics, scatter plot of planning time vs path length

**Dependencies:** `scipy` (for statistical tests), `matplotlib` (for plots), `tkinter` (for GUI)

## Map Generation

### Creating a New Map with Simple Mapper

#### Terminal 1 – Launch Simple Mapper

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models:~/ros2_ws/src/crazyflie-simulation/simulator_files/gazebo
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py world:=room2_world.sdf
```

#### Terminal 2 – Start Keyboard Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use arrow keys to drive the drone and build the map.

#### Terminal 3 – Save the Map

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/cf_room2_map --ros-args -r map:=/map
```

### Using Your Custom Map

Once saved, use your custom map in navigation:

```bash
ros2 launch crazyflie_ros2_multiranger_bringup navigation_simulation.launch.py \
    spawn_x:=-5.0 spawn_y:=-1.0 spawn_yaw:=0
```

### Regenerating the Procedural Maze

To regenerate a maze with a different seed:

```bash
python3 ~/ros2_ws/scripts/generate_maze.py
```

## Navigation And Benchmark Data

The repository includes benchmark inputs and outputs from path planning experiments:

- `algo10each.csv` and `algos4-5each.csv` – Comparison data for planning runs
- `runAnalysisText.txt` – Statistical analysis output across algorithms (A*, Dijkstra, GA, RRT)
- `public_fyp_repo/reports/` – Per-run JSON reports for all planning runs
- `public_fyp_repo/maps/` – Occupancy-grid maps (PGM format) and YAML metadata

These artifacts demonstrate comparisons of path length, distance traveled, smoothness, planning time, and execution time across multiple planning strategies.

## License And Attribution

This repository includes material adapted from the Crazyflie and Crazyswarm2 ecosystems. Refer to the nested project documentation and source headers for upstream attribution and simulator-specific licensing details.
