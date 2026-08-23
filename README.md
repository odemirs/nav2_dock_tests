# nav2_dock_tests

Simulation bringup for testing the Nav2 docking server.

## Requirements

Built against the **`feature/pluggable-docking-controller`** branch of
[odemirs/navigation2](https://github.com/odemirs/navigation2),
which makes the docking controller a pluginlib plugin.

Binary dependencies, tested for ROS 2 Jazzy:

- `nav2_minimal_tb3_sim` — TurtleBot3 Waffle model, `spawn_tb3.launch.py`, gz bridge config
- `ros_gz_sim`, `ros_gz_bridge` — Gazebo Sim 8 (Harmonic) integration
- `robot_state_publisher`, `xacro`, `rviz2`

## Build

Our setup uses ros2 jazzy for Ubuntu 24.04. The BehaiorTree library
provided by jazzy is not compatible with the main branch of
navigation2. So, it also build from source.

```bash
cd <path-to-your-workspace>
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
git checkout tags/4.10.0
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src
colcon build --symlink-install
```

```bash
cd <path-to-your-workspace>
git clone https://github.com/odemirs/navigation2.git
cd navigation2
git checkout feature/pluggable-docking-controller
source /opt/ros/jazzy/setup.bash
source <path-to-your-workspace>/BehaviorTree.CPP/install/local_setup.bash
rosdep install --from-paths . --ignore-src
colcon build --symlink-install
```

```bash
cd <path-to-your-workspace>
git clone https://github.com/odemirs/nav2_dock_tests.git
source /opt/ros/jazzy/setup.bash
source <path-to-your-workspace>/BehaviorTree.CPP/install/local_setup.bash
source <path-to-your-workspace>/navigation2/install/local_setup.bash
rosdep install --from-paths nav2_dock_tests --ignore-src
colcon build --packages-select nav2_dock_tests --symlink-install
source install/local_setup.bash
```

## Run

```bash
ros2 launch nav2_dock_tests dock_test_launch.py
```

With the Gazebo GUI:

```bash
ros2 launch nav2_dock_tests dock_test_launch.py headless:=False
```

The robot spawns at `(-2.0, -0.5)`, which matches the initial pose AMCL is seeded
with in `params/nav2_dock_tests_params.yaml`.

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `headless` | `True` | Run Gazebo without its GUI |

This is a fixed test bed rather than a general purpose bringup, so other params are
constants at the top of `launch/dock_test_launch.py`

| Constant | Value | |
|---|---|---|
| `USE_SIM_TIME` | `True` | |
| `AUTOSTART` | `True` | |
| `USE_COMPOSITION` | `True` | see caveat below |
| `USE_RESPAWN` | `False` | |
| `LOG_LEVEL` | `info` | |
| `USE_RVIZ` | `True` | |
| `SPAWN_X` / `SPAWN_Y` / `SPAWN_Z` | `-2.00` / `-0.50` / `0.01` | must match amcl's seeded `initial_pose` |
| `SPAWN_ROLL` / `SPAWN_PITCH` / `SPAWN_YAW` | `0.00` / `0.00` / `0.00` | |
| `ROBOT_NAME` | `turtlebot3_waffle` | |
| `DOCK_X` / `DOCK_Y` / `DOCK_YAW` | `2.0` / `-0.6` / `0.0` | must match `home_dock` in the parameter file |
| `DOCK_DETECTION_RATE` | `20.0` Hz | |
| `DOCK_NOISE_STDDEV_X` | `0.0` | Gaussian noise stddev on the detected dock x [m] |
| `DOCK_NOISE_STDDEV_Y` | `0.0` | Gaussian noise stddev on the detected dock y [m] |
| `DOCK_NOISE_STDDEV_YAW` | `0.0` | Gaussian noise stddev on the detected dock yaw [rad] |
| `DOCK_NOISE_SEED` | `0` | `0` is nondeterministic |

## Layout

```
launch/dock_test_launch.py     the one launch file: simulation + localization + navigation + RViz
nodes/dock_pose_publisher.py   synthetic detected_dock_pose, with configurable Gaussian noise
params/                        one parameter file for the whole stack
maps/                          occupancy grid matching the world
worlds/                        Gazebo world (xacro, so the GUI plugin is conditional)
models/simple_dock/            the dock: a plain static box
rviz/                          RViz configuration
```

## The dock

`models/simple_dock` is a static 0.30 x 0.40 x 0.20 m box, placed at `(2.42, -0.60)`.

Dock with:

```bash
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: home_dock, navigate_to_staging_pose: true}"
```

### Dock detection

`nodes/dock_pose_publisher.py` publishes a synthetic `detected_dock_pose`, which is what
`SimpleChargingDock::getRefinedPose` consumes when `use_external_detection_pose` is `true`.

## Deviation from upstream nav2_params.yaml

The parameter file originated from `nav2_bringup/params/nav2_params.yaml`.
