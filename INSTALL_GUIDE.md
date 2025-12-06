# ROS2 Installation Guide

## Quick Install

Run the installation script:

```bash
cd /home/collab2/giuse/dt_durability
./install_ros2.sh
```

This will install ROS2 Foxy Desktop which includes:
- `rclpy` - ROS2 Python client library
- `rviz2` - 3D visualization tool
- ROS2 CLI tools
- Example programs and demos

## After Installation

### 1. Add ROS2 to your shell
```bash
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 2. Verify installation
```bash
# Check ROS2 command
ros2 --help

# Check rclpy
python3 -c "import rclpy; print('rclpy works!')"
```

### 3. Run the durability test
```bash
cd /home/collab2/giuse/dt_durability
python3 cnc_durability_ros.py
```

## Installation Details

The script installs:
- **ROS2 Foxy Desktop** (~1.5GB)
  - Location: `/opt/ros/foxy/`
  - Includes: rclpy, rviz2, common tools

- **Development Tools**
  - python3-argcomplete
  - python3-colcon-common-extensions

## Troubleshooting

### If installation fails

**Check your Ubuntu version:**
```bash
lsb_release -a
# Should show: Ubuntu 20.04
```

**Check disk space:**
```bash
df -h
# Need at least 2GB free
```

### If rclpy import fails

```bash
# Make sure to source ROS2 setup
source /opt/ros/foxy/setup.bash

# Then try again
python3 -c "import rclpy"
```

### Alternative: Minimal Installation

If you want a lighter install (no GUI tools):

```bash
sudo apt update
sudo apt install -y ros-foxy-ros-base python3-argcomplete
source /opt/ros/foxy/setup.bash
```

This installs only the core ROS2 libraries (~500MB vs ~1.5GB).

## Uninstallation

To remove ROS2:

```bash
sudo apt remove ros-foxy-* && sudo apt autoremove
sudo rm /etc/apt/sources.list.d/ros2.list
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
```

## Next Steps

After installation, see:
- `README_ROS.md` - How to use the ROS2 durability testing system
- `IMPLEMENTATION_SUMMARY.md` - Technical details of the ROS2 implementation

## Additional Resources

- ROS2 Foxy Documentation: https://docs.ros.org/en/foxy/
- ROS2 Tutorials: https://docs.ros.org/en/foxy/Tutorials.html
- rclpy API: https://docs.ros2.org/foxy/api/rclpy/
