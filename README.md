# Hexapod Robot Project

A ROS2 implementation of a six-legged hexapod robot with walking locomotion control.

## Overview

This project contains a complete ROS2 package suite for simulating and controlling a hexapod robot. The hexapod features 6 legs with 3 joints each (coxa, femur, tibia) for a total of 18 degrees of freedom.

## Packages

### hexapod_model_description
- **Purpose**: URDF model definition and visualization tools
- **Features**: 
  - Complete hexapod URDF with 18 joints
  - Gazebo simulation support
  - RViz visualization launch files
  - STL mesh files for all robot parts

### hexapod_control
- **Purpose**: Walking control algorithms and joint state management
- **Features**:
  - Tripod gait walking controller
  - Real-time joint state publishing
  - Configurable walking parameters

## Quick Start

### Prerequisites
- ROS2 Jazzy
- Python 3
- Gazebo (optional, for simulation)
- RViz2 (for visualization)

### Build the Workspace
```bash
cd ~/ros2_ws
colcon build --packages-select hexapod_model_description hexapod_control
source install/setup.zsh  # or setup.bash
```

### Launch Robot Visualization
```bash
ros2 launch hexapod_model_description display.launch.py
```

### Start Walking Controller
```bash
python3 ~/ros2_ws/src/hexapod_control/hexapod_control/walking_controller.py
```

## Robot Specifications

- **Legs**: 6 legs arranged in radial pattern
- **Joints per leg**: 3 (coxa, femur, tibia)
- **Total DOF**: 18 joints
- **Gait**: Tripod gait pattern
- **Control**: Joint position control via `/joint_states` topic

## Joint Configuration

Each leg follows the naming convention:
- `coxa_X_joint`: Hip rotation joint
- `femur_X_joint`: Upper leg joint  
- `tibia_X_joint`: Lower leg joint

Where X is the leg number (1-6).

### Default Resting Positions
- Femur: 0.5 radians
- Tibia: -1.5 radians  
- Coxa: 0.0 radians

## License

This project is provided as-is for educational and research purposes.
