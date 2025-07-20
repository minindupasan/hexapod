# Hexapod Robot - ROS2 Jazzy Implementation

This implementation brings functional concepts from the jethexa project to ROS2 Jazzy with proper gz sim integration and ros2_control framework.

## Features

- **ROS2 Jazzy Compatible**: Uses latest ROS2 standards and best practices
- **Gazebo Integration**: Full simulation with gz sim and ros_gz_bridge
- **ROS2 Control**: Proper ros2_control framework integration
- **Joint Position Control**: 18-DOF hexapod with position controllers
- **Web Interface**: Optional rosbridge for web-based control
- **Walking Gait**: Tripod gait implementation with ROS2 control

## Architecture

### Core Components

1. **Robot Description** (`hexapod_model_description`)

   - URDF with ros2_control tags
   - Gazebo configuration with proper physics
   - RViz visualization

2. **Control System** (`hexapod_control`)

   - ROS2 Control configuration
   - Joint position controllers
   - Walking gait algorithms

3. **Simulation** (`hexapod_bringup`)
   - Gazebo simulation launch
   - ROS-Gazebo bridge configuration
   - Complete system integration

### Key Technologies

- **gz sim** (Gazebo): Physics simulation
- **ros_gz_bridge**: ROS2-Gazebo communication
- **ros2_control**: Hardware abstraction and control
- **controller_manager**: Controller lifecycle management
- **position_controllers**: Joint position control

## Installation

### Prerequisites

```bash
# ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop

# Gazebo
sudo apt install gz-harmonic

# ROS2 Control
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# Gazebo-ROS2 Bridge
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge

# Additional packages
sudo apt install ros-jazzy-teleop-twist-keyboard
sudo apt install ros-jazzy-rosbridge-server
```

### Build the Workspace

```bash
cd ~/ros2_ws
colcon build --packages-select hexapod hexapod_model_description hexapod_control hexapod_bringup
source install/setup.bash
```

## Usage

### 1. Basic Simulation

Start the complete hexapod simulation:

```bash
ros2 launch hexapod_bringup gazebo.launch.py
```

This will launch:

- Gazebo simulation
- Robot state publisher
- ROS-Gazebo bridge
- Controller manager
- RViz visualization

### 2. Walking Controller

In a new terminal, start the walking controller:

```bash
ros2 run hexapod_control ros2_control_walking_controller.py
```

### 3. Manual Control

Control the hexapod with keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keyboard commands to move the hexapod:

- `i`: Forward
- `k`: Stop
- `j`: Turn left
- `l`: Turn right

### 4. Complete System

Launch everything together:

```bash
ros2 launch hexapod_bringup hexapod_complete.launch.py
```

### 5. Web Interface (Optional)

Enable web control interface:

```bash
ros2 launch hexapod_bringup hexapod_complete.launch.py enable_bridge:=true
```

Then connect to `ws://localhost:9090` with a web client.

## Testing

### Test Joint Movement

Run the test controller to verify joint movement:

```bash
ros2 run hexapod_control test_controller_ros2.py
```

### Monitor Topics

Check available topics:

```bash
ros2 topic list
```

Key topics:

- `/joint_states` - Joint state feedback
- `/hexapod_position_controller/commands` - Position commands
- `/cmd_vel` - Velocity commands

### Controller Status

Check controller status:

```bash
ros2 control list_controllers
```

## Configuration

### Controllers

Edit `hexapod_control/config/hexapod_controllers.yaml` to modify:

- Control rates
- Joint limits
- Controller parameters

### Gazebo Bridge

Edit `hexapod/config/ros_gz_bridge_gazebo.yaml` to configure:

- Topic bridging
- Message types
- Communication channels

### Walking Parameters

Modify walking gait in `ros2_control_walking_controller.py`:

- Step height and length
- Gait timing
- Body height

## Troubleshooting

### Common Issues

1. **Controllers not loading**

   ```bash
   ros2 control load_controller joint_state_broadcaster
   ros2 control set_controller_state joint_state_broadcaster active
   ```

2. **Gazebo not spawning robot**

   - Check URDF syntax: `check_urdf hexapod_model.urdf.xacro`
   - Verify package paths in launch files

3. **Bridge not working**
   - Check bridge configuration file
   - Verify topic names match between ROS2 and Gazebo

### Debug Commands

```bash
# Check joint states
ros2 topic echo /joint_states

# Monitor controller commands
ros2 topic echo /hexapod_position_controller/commands

# List active controllers
ros2 control list_controllers

# Check bridge status
ros2 topic list | grep gz
```

## Development

### Adding New Controllers

1. Create controller configuration in `hexapod_controllers.yaml`
2. Add to controller manager launch
3. Implement controller logic in Python/C++

### Extending Gaits

1. Modify `ros2_control_walking_controller.py`
2. Add new gait patterns
3. Tune parameters for stability

### Web Integration

The rosbridge server enables:

- Real-time joint control from web browsers
- Sensor data streaming
- Remote monitoring and control

## License

This project adapts concepts from the jethexa project for ROS2 Jazzy while maintaining original naming conventions and robot structure.
