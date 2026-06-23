# Autonomous Driving CARLA Simulator

A ROS 2 workspace for autonomous driving research and sensor data collection using the [CARLA Simulator](https://carla.org/). The project provides a modular pipeline that connects to a running CARLA server, spawns an instrumented ego vehicle, and records multi-modal sensor data for downstream perception and planning development.

---

## Table of Contents

- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Workspace Setup](#workspace-setup)
- [Quick Start](#quick-start)
- [Packages](#packages)
- [External Dependencies](#external-dependencies)

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   CARLA Simulator                        │
│           (running separately on port 2000)              │
└──────────────────────────┬──────────────────────────────┘
                           │ TCP
                           ▼
┌─────────────────────────────────────────────────────────┐
│              ROS 2 Workspace  (this repo)                │
│                                                          │
│   ┌──────────────────────────────────────────────────┐  │
│   │  ads_pipeline  ·  SensorManager node             │  │
│   │  ├── RGB Camera  →  /carla/camera/rgb/image      │  │
│   │  ├── LiDAR       →  /carla/lidar/points          │  │
│   │  └── IMU         →  /carla/imu/data              │  │
│   └──────────────────────┬───────────────────────────┘  │
│                          │ ros2 bag record               │
│                          ▼                               │
│            ~/recorded_sensor_bag/                        │
│            sensor_bag_<YYYYMMDD_HHMMSS>/                 │
└─────────────────────────────────────────────────────────┘
```

The pipeline is intentionally single-entry: one `ros2 launch` command starts the sensor node and the bag recorder simultaneously. Future packages (perception, planning, control) will plug in as additional nodes that subscribe to the same topics.

---

## Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 (Humble) or 24.04 (Jazzy) |
| ROS 2 | Humble or Jazzy |
| CARLA Simulator | 0.9.13 or later |
| CARLA Python API | matching CARLA server version |
| Python | 3.10+ |

Install ROS 2 dependencies:

```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge \
                 ros-$ROS_DISTRO-sensor-msgs \
                 ros-$ROS_DISTRO-nav-msgs \
                 ros-$ROS_DISTRO-tf2-ros
```

The CARLA Python API (`carla` package) must be on your `PYTHONPATH`. Refer to the [CARLA Python API documentation](https://carla.readthedocs.io/en/latest/python_api/) for installation instructions.

---

## Workspace Setup

```bash
# Clone the repository into your colcon workspace
cd ~/carla_ws/src
git clone <repository-url> Autonomous_driving_Carla_simulator

# Build all packages
cd ~/carla_ws
colcon build

# Source the workspace
source install/setup.bash
```

Add the source line to your shell profile to avoid running it every session:

```bash
echo "source ~/carla_ws/install/setup.bash" >> ~/.bashrc
```

---

## Quick Start

1. **Start the CARLA server** (from your CARLA installation directory):

    ```bash
    ./CarlaUE4.sh
    ```

2. **Launch the full sensor pipeline** (vehicle spawn + sensor publishing + bag recording):

    ```bash
    ros2 launch ads_pipeline record_launch.py
    ```

3. **Stop with `Ctrl+C`** — the bag is finalized automatically.

Recorded bags are saved to `~/recorded_sensor_bag/sensor_bag_<timestamp>/`.

---

## Packages

| Package | Description | README |
|---|---|---|
| `ads_pipeline` | Core sensor pipeline — spawns ego vehicle, publishes RGB camera, LiDAR, and IMU data, and records to a ROS 2 bag. | [ads_pipeline/README.md](ads_pipeline/README.md) |

> Additional packages (perception, planning, control) will be added here as the project grows.

---

## External Dependencies

**[`carla_ros2_bridge`](../carla_ros2_bridge/)** is a sibling repository (located at `src/carla_ros2_bridge/`) that contains prototype scripts developed prior to `ads_pipeline`. It includes:

- A standalone camera publisher node with odometry and coordinate-frame corrections
- A non-ROS sensor validation script with LiDAR-to-camera projection
- CARLA map exploration and weather experiment utilities

These scripts are not required to run the pipeline but serve as reference implementations. See the `carla_ros2_bridge` README for details.
