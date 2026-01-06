# 🤖 ROS2 Autonomous Driving and Navigation SLAM with TurtleBot3

```
╔═══════════════════════════════════════════════════════════════════╗
║                   TURTLEBOT3 AUTONOMOUS NAVIGATION                ║
║              SLAM | Navigation | Path Planning | Mapping           ║
╚═══════════════════════════════════════════════════════════════════╝
```

A comprehensive ROS2 package for autonomous navigation, SLAM (Simultaneous Localization and Mapping), and navigation of the TurtleBot3 platform in various environments.

## 📋 Project Overview

This project implements an autonomous robot system using TurtleBot3 with ROS2. It provides multiple functionality modules for:

```
┌─────────────────────────────────────────────────────────────┐
│                    CORE FEATURES                            │
├─────────────────────────────────────────────────────────────┤
│ 🗺️  SLAM                   → Cartographer SLAM Stack        │
│ 🧭 Autonomous Navigation   → Nav2 Stack                     │
│ 🏗️  Gazebo Simulation      → Environment Integration        │
│ 📊 RViz2 Visualization     → Real-time Monitoring           │
└─────────────────────────────────────────────────────────────┘
```

## 📂 Project Structure

```
ROS2-Autonomous-Driving-and-Navigation-SLAM-with-TurtleBot3/
│
└── navigation_turtlebot/
    │
    ├── 🐍 autonomous_tb3/                 [Python Package]
    │   ├── __init__.py
    │   ├── occupancy_grid_pub.py          ← Publishes /map topic (OccupancyGrid)
    │   └── spawn_entity.py                ← Spawns Gazebo models dynamically
    │
    ├── ⚙️  config/                        [Configuration Files]
    │   ├── tb3_cartographer.lua           ← SLAM algorithm tuning
    │   ├── tb3_nav_params.yaml            ← AMCL & planner parameters
    │   ├── tb3_nav.rviz                   ← RViz2 visualization setup
    │   ├── tb3_world.pgm                  ← Map image
    │   └── tb3_world.yaml                 ← Map metadata
    │
    ├── 🚀 launch/                         [ROS2 Launch Files]
    │   ├── mapping.launch.py              ← Start Cartographer SLAM
    │   ├── maze_navigation.launch.py      ← Full maze navigation setup
    │   └── tb3_world_navigation.launch.py ← TurtleBot3 world navigation
    │
    ├── 📦 resource/                       [Package Resources]
    ├── ✅ test/                           [Unit Tests]
    ├── 📄 package.xml                     [ROS2 Package Metadata]
    ├── 📄 setup.py                        [Python Setup]
    └── 📄 setup.cfg                       [Setup Configuration]
```🔧 Key Components

### 1. 🚀 Launch Files

#### `mapping.launch.py` - SLAM Cartographer
```
┌──────────────────────────────────────┐
│     Cartographer SLAM Stack          │
├──────────────────────────────────────┤
│ • Cartographer Node (SLAM)           │
│ • Occupancy Grid Node (Map Output)   │
└──────────────────────────────────────┘
```

#### `maze_navigation.launch.py` - Full Maze Navigation
```
┌────────────────────────────────────────────────────────────┐
│              MAZE NAVIGATION PIPELINE                      │
├────────────────────────────────────────────────────────────┤
│  Gazebo              Robot              Mapping            │
│  ├─ gzserver        ├─ State Pub       └─ SLAM Toolbox   │
│  └─ gzclient        └─ Spawner              ↓             │
│       ↓                  ↓            Nav2 Navigation      │
│   Simulation        TurtleBot3         ├─ Planner         │
│                                        ├─ Controller      │
│                                        └─ BehaviorTrees   │
│                                             ↓             │
│                                        RViz2 Viz          │
└────────────────────────────────────────────────────────────┘
```

**Launch Arguments:**
```yaml
x_pose: -5.2  # Robot initial X position
y_pose: -6.7  # Robot initial Y position
```

#### `tb3_world_navigation.launch.py` - Pre-mapped Navigation
```
┌──────────────────────────────────────────┐
│    TB3 WORLD NAVIGATION                  │
├──────────────────────────────────────────┤
│ TurtleBot3        │  Nav2 Stack  │ RViz2│
│ Gazebo Sim        │  (Pre-map)   │      │
│ Robot Model       │  AMCL        │      │
│                   │  Planner     │      │
└──────────────────────────────────────────┘
```
- Nav2 bringup with pre-loaded map
- RViz2 with custom navigation configuration

### 2. 🐍 Python Modules

#### `occupancy_grid_pub.py` - Map Publisher
```
┌─────────────────────────────────────────┐
│  OCCUPANCY GRID PUBLISHER NODE          │
├─────────────────────────────────────────┤
│ Timer Callback (0.5 Hz)                 │
│         ↓                               │
│ Create OccupancyGrid Message            │
│  ├─ Header (timestamp, frame_id)       │
│  ├─ Info (resolution, width, height)   │
│  └─ Data (3x3 grid, int8 values)       │
│         ↓                               │
│ Publish → /map topic                    │
└─────────────────────────────────────────┘

Entry Point: occupancy_grid_pub
Frequency: 0.5 Hz (every 0.5 seconds)
Topic: /map
Message Type: OccupancyGrid
```

#### `spawn_entity.py` - Gazebo Model Spawner
```
┌─────────────────────────────────────────┐
│  GAZEBO MODEL SPAWNER                   │
├─────────────────────────────────────────┤
│ Input Arguments:                        │
│  • SDF file path                        │
│  • Entity name                          │
│  • Position (X, Y, Z) [optional]        │
│         ↓                               │
│ Connect to /spawn_entity service        │
│         ↓                               │
│ Create SpawnEntity.Request              │
│         ↓                               │
│ Call Gazebo Service → Spawned Model     │
└─────────────────────────────────────────┘

Entry Point: sdf_spawner
Service: /spawn_entity
Example: ros2 run autonomous_tb3 sdf_spawner model.sdf my_entity 0.0 0.0
```

### 3. ⚙️ Configuration Files

#### `tb3_nav_params.yaml` - Navigation Parameters
```yaml
Navigation Stack Parameters:

AMCL (Adaptive Monte Carlo Localization)
├── Particles: [500 min, 2000 max]
├── Laser Model: likelihood_field
├── Max Beams: 60
├── Frames:
│   ├── Global: map
│   ├── Base: base_footprint
│   └── Odom: odom
└── Sensor Range: -1.0 to 100.0 m

Motion Model:
└── Differential Motion (TurtleBot3)
```

#### `tb3_cartographer.lua` - SLAM Configuration
```
Cartographer SLAM Tuning:
├── Sensor Configuration
├── Trajectory Builder Options
├── Motion Filter Settings
├── Scan Matcher Configuration
└── Optimization Parameters
```

### 4. 📊 Maps & Resources

| File | Purpose | Format |
|------|---------|--------|
| `tb3_world.pgm` | Occupancy grid map image | PGM (Portable GrayMap) |
| `tb3_world.yaml` | Map metadata & origin | YAML |
| `maze.yaml` | Maze environment map | YAML |

## 📦 Dependencies

### ROS2 Packages
```
Navigation Stack:
  ├── nav2_bringup ──────────────── Navigation Framework
  ├── cartographer_ros ──────────── SLAM Mapping
  └── slam_toolbox ──────────────── Alternative SLAM

Robot & Simulation:
  ├── turtlebot3_gazebo ────────── Robot Model & World
  └── gazebo_ros ────────────────── Gazebo Integration

Visualization:
  └── rviz2 ──────────────────────── 3D Visualization

Core Dependencies:
  └── rclpy ──────────────────────── ROS2 Python API
```

### Python Dependencies
```python
rclpy              # ROS2 Python client library
setuptools         # Package management
numpy              # Numerical computing
```

## 🔧 Installation

### Prerequisites
```
✓ ROS2 (latest distribution)
✓ Python 3.8+
✓ Gazebo Simulation Engine
✓ TurtleBot3 ROS2 packages
✓ Git
```

### Build & Setup

1️⃣ **Clone Repository**
```bash
cd ~/ros2_ws/src
git clone <repository-url>
cd ..
```

2️⃣ **Install Dependencies**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3️⃣ **Build Package**
```bash
colcon build --packages-select autonomous_tb3
```

4️⃣ **Source Environment**
```bash
source ~/ros2_ws/install/setup.bash
```

## 🚀 Usage

### 🎯 Maze Navigation (Full Setup)
```bash
ros2 launch autonomous_tb3 maze_navigation.launch.py
```
**Launches:** Gazebo + SLAM + Nav2 + RViz2
**Use Case:** Autonomous maze solving with real-time mapping

### 🗺️ TurtleBot3 World Navigation (Pre-mapped)
```bash
ros2 launch autonomous_tb3 tb3_world_navigation.launch.py
```
**Launches:** TurtleBot3 + Pre-loaded map + Nav2 + RViz2
**Use Case:** Navigation in pre-known environment

### 📡 SLAM Mapping Only
```bash
ros2 launch autonomous_tb3 mapping.launch.py
```
**Launches:** Cartographer SLAM stack
**Use Case:** Building and testing maps

### 📊 Occupancy Grid Publishing
```bash
ros2 run autonomous_tb3 occupancy_grid_pub
```
**Output:** Publishes to `/map` topic at 0.5 Hz
**Message Type:** nav_msgs/OccupancyGrid

### 🎮 Spawn Custom Models
```bash
# Syntax
ros2 run autonomous_tb3 sdf_spawner <sdf_file> <model_name> [x] [y]

# Example
ros2 run autonomous_tb3 sdf_spawner /path/to/model.sdf my_model 0.0 0.0
```
**Service:** Calls Gazebo `/spawn_entity` service

## 📊 RViz2 Visualization

```
┌─────────────────────────────────────────────┐
│         RVIZ2 VISUALIZATION DISPLAY          │
├─────────────────────────────────────────────┤
│ ✓ Robot TF Frame Tree                       │
│ ✓ Costmaps (Local & Global)                 │
│ ✓ Laser Scan Data                           │
│ ✓ Occupancy Grid (Static Map)               │
│ ✓ Navigation Goals & Planned Paths          │
│ ✓ Particle Cloud (AMCL)                     │
│ ✓ Robot Model (URDF Visualization)          │
└─────────────────────────────────────────────┘
```

**Configuration File:** `config/tb3_nav.rviz`

Both navigation launches automatically open RViz2 with pre-configured views.

## 🧭 Navigation Stack Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                  NAVIGATION PIPELINE                         │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  PERCEPTION              LOCALIZATION       PLANNING        │
│  ───────────────────────────────────────────────────────    │
│                                                              │
│  Sensor Inputs           AMCL Node          Planner         │
│    ├─ Laser Range    ──→  ├─ Particle Filter ──→ GlobalPath│
│    ├─ Odometry       ──→  ├─ Pose Estimate     └─ CostMap  │
│    └─ IMU                 └─ Uncertainty                    │
│                                                 ↓            │
│                           Controllers & BehaviorTrees       │
│                                 ↓                           │
│                           Motor Commands → Robot Motion     │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### AMCL (Adaptive Monte Carlo Localization)
```
Particle Filter Based Localization:
├─ Initial Pose Distribution
├─ Particle Weight Update (Laser Likelihood)
├─ Resampling (Low Weight Removal)
├─ Motion Update (Odometry Integration)
└─ Output: Estimated Robot Pose & Uncertainty

Configuration:
  • Max Particles: 2000
  • Min Particles: 500
  • Update Rate: Laser-triggered
  • Laser Model: Likelihood Field
```

### Nav2 Navigation Stack
```
Navigation2 Components:
├─ Planner: Path Planning (Global)
├─ Controller: Motion Control (Local)
├─ Recovery: Stuck Detection & Recovery
├─ BehaviorTree: Task Sequencing
└─ CostMaps: Obstacle Representation
```

## ✅ Testing

Standard ROS2 package testing suite:

```
Test Categories:
├─ test_copyright.py      → Verify Apache 2.0 license headers
├─ test_flake8.py         → Code style and PEP8 compliance
└─ test_pep257.py         → Docstring conventions
```

**Run All Tests:**
```bash
colcon test --packages-select autonomous_tb3
colcon test-result --verbose
```

**Run Specific Test:**
```bash
colcon test --packages-select autonomous_tb3 --ctest-args -R test_flake8
```

## 🎯 Entry Points (Console Scripts)

```
ROS2 Executables Provided:

occupancy_grid_pub
  └─ Publishes OccupancyGrid messages to /map
  
sdf_spawner
  └─ Spawns Gazebo models from SDF files
  
maze_solver
  └─ Autonomous maze solving algorithm
  
autonomous_waiter_lite
  └─ Single goal navigation demo
  
autonomous_waiter
  └─ Multi-goal navigation demo
```

**Package Metadata:**
```yaml
Name:        autonomous_tb3
Version:     0.0.0
Maintainer:  Luqman (noshluk2@gmail.com)
Build Type:  ament_python
Format:      ROS2 Package Format 3
```

## 📝 Notes

```
Project Highlights:

✓ Dual Map Support
  ├─ tb3_world.yaml (Standard world navigation)
  └─ maze.yaml (Maze solving)

✓ Comprehensive Gazebo Integration
  ├─ Actor models for dynamic obstacles
  ├─ Table and object models
  ├─ Beer/item models with textures
  └─ Custom scripts for simulation

✓ Production-Ready SLAM
  ├─ Cartographer algorithm (proven performance)
  ├─ SLAM Toolbox (alternative implementation)
  └─ Real-time occupancy grid generation

✓ Hardware Support
  ├─ Real Robot Deployment (TurtleBot3 Burger/Waffle)
  └─ Full Simulation Environment
```

## 🚀 Future Development

```
Roadmap & Enhancements:

Phase 1 - Documentation
  □ Complete license declaration
  □ Add comprehensive API documentation
  □ Create video tutorials

Phase 2 - Features
  □ Multi-robot coordination
  □ Dynamic obstacle avoidance
  □ Semantic mapping
  □ Human-robot interaction

Phase 3 - Optimization
  □ Performance tuning for real hardware
  □ CPU/GPU optimization
  □ Battery life optimization
  □ Enhanced sensor integration

Phase 4 - Advanced
  □ Machine learning integration
  □ Predictive navigation
  □ Social robot behaviors
  □ Cloud robotics support
```

## License

License declaration to be determined (see package.xml).

## Contact

For questions or contributions, contact: noshluk2@gmail.com
