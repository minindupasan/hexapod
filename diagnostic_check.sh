#!/bin/bash

echo "🔍 HEXAPOD SIMULATION DIAGNOSTIC TOOL"
echo "====================================="
echo ""

# System Information
echo "📊 SYSTEM INFORMATION:"
echo "OS: $(lsb_release -d | cut -f2)"
echo "Kernel: $(uname -r)"
echo "Architecture: $(uname -m)"
echo "RAM: $(free -h | grep '^Mem:' | awk '{print $2}')"
echo "CPU: $(lscpu | grep 'Model name' | cut -d: -f2 | xargs)"
echo ""

# Graphics Information
echo "🎮 GRAPHICS INFORMATION:"
echo "GPU: $(lspci | grep -i vga | cut -d: -f3 | xargs)"
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA Driver: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader,nounits)"
fi
echo "OpenGL Version: $(glxinfo | grep "OpenGL version" | cut -d: -f2 | xargs)"
echo ""

# ROS2 Environment
echo "🤖 ROS2 ENVIRONMENT:"
echo "ROS_DISTRO: ${ROS_DISTRO:-'NOT SET'}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-'0 (default)'}"
echo "ROS2 Installation: $(which ros2 || echo 'NOT FOUND')"
if [ ! -z "$ROS_DISTRO" ]; then
    echo "ROS2 Version: $(ros2 --version 2>/dev/null || echo 'Error getting version')"
fi
echo ""

# Gazebo Packages
echo "🌍 GAZEBO PACKAGES:"
echo "Classic Gazebo packages:"
ros2 pkg list | grep gazebo || echo "  No classic gazebo packages found"
echo ""
echo "Modern Gazebo (gz) packages:"
ros2 pkg list | grep "^gz" || echo "  No gz packages found"
echo ""
echo "ROS-Gazebo bridge packages:"
ros2 pkg list | grep "ros_gz" || echo "  No ros_gz packages found"
echo ""

# Dependencies Check
echo "🔧 DEPENDENCIES CHECK:"
echo "Required packages:"
packages=("xacro" "robot_state_publisher" "joint_state_publisher" "rviz2")
for pkg in "${packages[@]}"; do
    if ros2 pkg list | grep -q "^$pkg$"; then
        echo "  ✅ $pkg - Found"
    else
        echo "  ❌ $pkg - Missing"
    fi
done
echo ""

# Python Dependencies
echo "🐍 PYTHON DEPENDENCIES:"
python_deps=("numpy" "scipy" "rclpy")
for dep in "${python_deps[@]}"; do
    if python3 -c "import $dep" 2>/dev/null; then
        version=$(python3 -c "import $dep; print($dep.__version__)" 2>/dev/null || echo "unknown")
        echo "  ✅ $dep - $version"
    else
        echo "  ❌ $dep - Missing"
    fi
done
echo ""

# Workspace Check
echo "📁 WORKSPACE CHECK:"
if [ -f "~/ros2_ws/install/setup.bash" ]; then
    echo "  ✅ Workspace built"
    echo "  Packages in workspace:"
    ls ~/ros2_ws/install/ | grep -v "^_" | head -10
else
    echo "  ❌ Workspace not built or not found"
fi
echo ""

# Network Check
echo "🌐 NETWORK CHECK:"
echo "Hostname: $(hostname)"
echo "IP Address: $(hostname -I | awk '{print $1}')"
echo "Localhost resolution: $(ping -c1 localhost 2>/dev/null && echo 'OK' || echo 'FAILED')"
echo ""

# Gazebo Test
echo "🧪 GAZEBO TEST:"
if command -v gz &> /dev/null; then
    echo "  Modern Gazebo (gz) found: $(gz --version 2>/dev/null | head -1)"
    echo "  Testing gz sim..."
    timeout 5s gz sim --help &>/dev/null && echo "  ✅ gz sim works" || echo "  ❌ gz sim failed"
else
    echo "  ❌ Modern Gazebo (gz) not found"
fi

if command -v gazebo &> /dev/null; then
    echo "  Classic Gazebo found: $(gazebo --version 2>/dev/null | head -1)"
else
    echo "  ❌ Classic Gazebo not found"
fi
echo ""

echo "🏁 DIAGNOSTIC COMPLETE"
echo "Run this script on both devices and compare the outputs."
echo "Focus on differences in:"
echo "  - ROS2 versions and packages"
echo "  - Gazebo installation"
echo "  - Graphics drivers"
echo "  - Missing dependencies"
