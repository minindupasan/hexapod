# 🕷️ Hexapod Robot Project

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?style=for-the-badge&logo=ros)
![Python](https://img.shields.io/badge/Python-3.8+-green?style=for-the-badge&logo=python)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange?style=for-the-badge)
![License](https://img.shields.io/badge/License-Apache%202.0-yellow?style=for-the-badge)

**A complete ROS2 implementation of a six-legged hexapod robot with advanced walking locomotion control**

[🚀 Quick Start](#-quick-start) • [📦 Packages](#-packages) • [🎮 Usage](#-usage) • [🤝 Contributing](#-contributing)

</div>

---

## 🌟 Overview

This project contains a comprehensive ROS2 package suite for simulating and controlling a hexapod robot. The hexapod features **6 legs** with **3 joints each** (coxa, femur, tibia) for a total of **18 degrees of freedom**, enabling complex walking patterns and terrain navigation.

### ✨ Key Features

- 🦾 **18 DOF locomotion** with tripod gait walking
- 🎯 **Real-time simulation** in Gazebo
- 🎨 **3D visualization** in RViz2
- 🧠 **Intelligent control algorithms**
- 📊 **Live joint state monitoring**
- 🔧 **Modular and extensible design**

---

## 📦 Packages

<table>
<tr>
<td>

### 🤖 hexapod_model_description

**Robot Definition & Visualization**

- ✅ Complete URDF model with 18 joints
- ✅ Gazebo simulation integration
- ✅ RViz visualization tools
- ✅ High-quality STL mesh files
- ✅ Physics-based collision models

</td>
<td>

### 🎮 hexapod_control

**Locomotion & Control**

- ✅ Advanced tripod gait algorithms
- ✅ Real-time joint state publishing
- ✅ Configurable walking parameters
- ✅ Teleop control interface
- ✅ Custom rotation utilities

</td>
</tr>
<tr>
<td>

### 🌍 hexapod_gz

**Gazebo Integration**

- ✅ Modern Gazebo (Ignition) support
- ✅ World files and environments
- ✅ Launch file configurations
- ✅ RViz integration
- 🚧 SLAM capabilities (planned)

</td>
<td>

### 🗺️ hexapod_navigation

**Path Planning & SLAM**

- 🚧 Navigation stack integration (planned)
- 🚧 SLAM toolbox configuration (planned)
- 🚧 Path planning algorithms (planned)
- 🚧 Obstacle avoidance (planned)
- 🚧 Autonomous navigation (planned)

</td>
</tr>
</table>

---

## 🚀 Quick Start

> [!NOTE]
> This guide assumes you have Ubuntu 22.04 LTS. For other distributions, adjust package installation commands accordingly.

### 📋 Prerequisites

> [!IMPORTANT]
> Make sure you have the following installed before proceeding:

#### 1. **ROS2 Jazzy Installation**

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop
```

#### 2. **Development Tools**

```bash
# Install build tools
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pip

# Install additional dependencies
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-rviz2
```

#### 3. **Initialize rosdep**

```bash
sudo rosdep init
rosdep update
```

### 🏗️ Building the Project

> [!TIP]
> Follow these steps carefully to set up your workspace:

#### 1. **Create and Setup Workspace**

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/minindupasan/hexapod.git .
```

#### 2. **Install Dependencies**

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Install package dependencies
rosdep install --from-paths src --ignore-src -r -y

# Source ROS2
source /opt/ros/jazzy/setup.bash
```

#### 3. **Build the Packages**

```bash
# Build all packages
colcon build

# Or build specific packages
colcon build --packages-select hexapod_model_description hexapod_control hexapod_gz

# Source the workspace
source install/setup.bash
```

> [!WARNING]
> If you encounter build errors, make sure all dependencies are installed and you've sourced ROS2 properly.

---

## 🎮 Usage

### 🤖 Robot Visualization

Launch the robot model in RViz for visualization:

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch robot visualization
ros2 launch hexapod_model_description display.launch.py
```

### 🌍 Gazebo Simulation

Start the complete simulation environment:

```bash
# Launch Gazebo simulation with robot
ros2 launch hexapod_gz complete.launch.py

# Launch with RViz visualization
ros2 launch hexapod_gz complete.launch.py use_rviz:=true
```

### 🚶 Walking Controller

Control the hexapod's walking motion:

```bash
# Start the walking controller
ros2 run hexapod_control hexapod_teleop.py

# Or use the walking controller directly
python3 ~/ros2_ws/src/hexapod_control/hexapod_control/walking_controller.py
```

### 🎯 Teleop Control

Control the robot using keyboard:

```bash
# Launch teleop interface
ros2 run hexapod_control hexapod_teleop.py
```

**Controls:**

- `W/S`: Forward/Backward
- `A/D`: Turn Left/Right
- `Space`: Stop
- `Q`: Quit

---

## 🔧 Configuration

### 🦿 Robot Specifications

<details>
<summary><strong>📊 Technical Details</strong></summary>

| Specification      | Value                    |
| ------------------ | ------------------------ |
| **Legs**           | 6 legs in radial pattern |
| **Joints per leg** | 3 (coxa, femur, tibia)   |
| **Total DOF**      | 18 joints                |
| **Gait Pattern**   | Tripod gait              |
| **Control Method** | Joint position control   |
| **Communication**  | ROS2 topics/services     |

</details>

### 🎛️ Joint Configuration

Each leg follows the naming convention:

```
Leg X (where X = 1-6):
├── coxa_X_joint   → Hip rotation joint
├── femur_X_joint  → Upper leg joint
└── tibia_X_joint  → Lower leg joint
```

#### Default Resting Positions

```yaml
femur_joints: 0.5 radians # Upper leg position
tibia_joints: -1.5 radians # Lower leg position
coxa_joints: 0.0 radians # Hip rotation
```

### 📡 ROS2 Topics

<details>
<summary><strong>🔗 Topic Reference</strong></summary>

| Topic                | Type                     | Description             |
| -------------------- | ------------------------ | ----------------------- |
| `/joint_states`      | `sensor_msgs/JointState` | Current joint positions |
| `/robot_description` | `std_msgs/String`        | Robot URDF description  |
| `/cmd_vel`           | `geometry_msgs/Twist`    | Velocity commands       |
| `/tf`                | `tf2_msgs/TFMessage`     | Transform tree          |

</details>

---

## 🛠️ Development

### 📁 Project Structure

```
hexapod/
├── 📁 hexapod_model_description/    # Robot model & visualization
│   ├── 📁 urdf/                     # URDF files
│   ├── 📁 meshes/                   # STL mesh files
│   ├── 📁 launch/                   # Launch files
│   └── 📁 config/                   # Configuration files
├── 📁 hexapod_control/              # Control algorithms
│   ├── 📁 hexapod_control/          # Python control modules
│   └── 📁 launch/                   # Control launch files
├── 📁 hexapod_gz/                   # Gazebo integration
│   ├── 📁 worlds/                   # Gazebo world files
│   ├── 📁 launch/                   # Simulation launch files
│   └── 📁 rviz/                     # RViz configurations
└── 📁 hexapod_navigation/           # Navigation & SLAM (planned)
    ├── 📁 config/                   # Navigation configs (planned)
    └── 📁 launch/                   # Navigation launch files (planned)
```

### 🧪 Testing

> [!TIP]
> Run tests to ensure everything works correctly:

```bash
# Run all tests
colcon test

# Run specific package tests
colcon test --packages-select hexapod_control

# View test results
colcon test-result --verbose
```

---

## 🤝 Contributing

We welcome contributions! Please see our contributing guidelines:

> [!NOTE]
>
> 1. Fork the repository
> 2. Create a feature branch (`git checkout -b feature/amazing-feature`)
> 3. Commit your changes (`git commit -m 'Add amazing feature'`)
> 4. Push to the branch (`git push origin feature/amazing-feature`)
> 5. Open a Pull Request

### 📝 Development Guidelines

- Follow ROS2 coding standards
- Add tests for new features
- Update documentation as needed
- Use meaningful commit messages

---

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

---

## 🆘 Troubleshooting

<details>
<summary><strong>🐛 Common Issues</strong></summary>

### Build Errors

> [!WARNING] > **Problem**: Package not found errors during build
>
> **Solution**:
>
> ```bash
> source /opt/ros/jazzy/setup.bash
> rosdep update
> rosdep install --from-paths src --ignore-src -r -y
> ```

### Gazebo Issues

> [!WARNING]
> **Problem**: Robot not appearing in Gazebo
>
> **Solution**:
>
> ```bash
> # Check if robot description is published
> ros2 topic echo /robot_description
>
> # Restart Gazebo
> killall gz
> ros2 launch hexapod_gz complete.launch.py
> ```

### Graphics Issues

> [!WARNING]
> **Problem**: Gazebo crashes or doesn't start
>
> **Solution**:
>
> ```bash
> # Check graphics drivers
> glxinfo | grep "OpenGL version"
> 
> # For Intel/AMD graphics, install Mesa drivers
> sudo apt install mesa-utils
> 
> # For NVIDIA, ensure proper drivers are installed
> nvidia-smi
> ```

### Transform Errors in RViz

> [!WARNING] > **Problem**: "No transform from [base_link] to [map]"
>
> **Solution**: Change RViz Fixed Frame to `base_link` instead of `map`

</details>

---

<div align="center">

### 🌟 Star this repository if you found it helpful!

Made with ❤️ by [Minindu Pasan](https://github.com/minindupasan)

</div>
