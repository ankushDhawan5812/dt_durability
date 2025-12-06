#!/bin/bash

# ROS2 Foxy Installation Script for Ubuntu 20.04
# This script installs ROS2 Foxy and rclpy

set -e  # Exit on error

echo "=========================================="
echo "ROS2 Foxy Installation Script"
echo "=========================================="
echo ""

# Check if running on Ubuntu 20.04
if ! grep -q "20.04" /etc/os-release; then
    echo "Warning: This script is designed for Ubuntu 20.04"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "[1/7] Setting up prerequisites..."
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release

echo ""
echo "[2/7] Adding ROS2 repository..."
sudo add-apt-repository universe -y

# Add ROS2 GPG key
echo "[3/7] Adding ROS2 GPG key..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository to sources list
echo "[4/7] Configuring ROS2 sources..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list
echo "[5/7] Updating package list..."
sudo apt update

# Install ROS2 Foxy Desktop (includes rclpy, rviz, demos, etc.)
echo "[6/7] Installing ROS2 Foxy Desktop (this may take a while)..."
sudo apt install -y ros-foxy-desktop

# Install development tools
echo "[7/7] Installing ROS2 development tools..."
sudo apt install -y python3-argcomplete python3-colcon-common-extensions

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""

# Source ROS2 setup
source /opt/ros/foxy/setup.bash

# Verify installation
echo "Verifying installation..."
if which ros2 > /dev/null 2>&1; then
    echo "✓ ROS2 command found"
else
    echo "✗ ROS2 command not found"
fi

if python3 -c "import rclpy" 2>/dev/null; then
    echo "✓ rclpy is installed"
else
    echo "✗ rclpy import failed"
fi

echo ""
echo "=========================================="
echo "Next Steps:"
echo "=========================================="
echo ""
echo "1. Add ROS2 to your shell startup:"
echo "   echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc"
echo "   source ~/.bashrc"
echo ""
echo "2. Test ROS2:"
echo "   ros2 --help"
echo "   python3 -c 'import rclpy; print(\"rclpy works!\")'"
echo ""
echo "3. Run your durability test:"
echo "   cd /home/collab2/giuse/dt_durability"
echo "   python3 cnc_durability_ros.py"
echo ""
echo "=========================================="
