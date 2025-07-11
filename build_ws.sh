#!/bin/bash

set -e

source ~/.bashrc

echo "======================================="
echo "Build NavOS workspace"
echo "======================================="

echo "[1/2] Building workspace with colcon (15 mins expected)..."
cd ~/NavOS
colcon build --symlink-install --executor sequential

SOURCE_LINE="source ~/NavOS/install/setup.bash"
if ! grep -Fxq "$SOURCE_LINE" ~/.bashrc; then
    echo "[2/2] Adding source line to ~/.bashrc..."
    echo "$SOURCE_LINE" >> ~/.bashrc
    echo "✅ NavOS path added to ~/.bashrc: $SOURCE_LINE"
else
    echo "✅ ~/.bashrc already contains the NavOS path."
fi

echo "======================================="
echo "✅ NavOS Installation Complete!"
echo "======================================="

echo ""
echo "======================================="
echo "Setting up sensor drivers workspace"
echo "======================================="

echo ""
echo "[1/5] Creating workspace at ~/sensors_ws/src..."
mkdir -p ~/sensors_ws/src
cd ~/sensors_ws/src

echo ""
echo "[2/5] Cloning RealSense ROS2 driver..."
git clone https://github.com/IntelRealSense/realsense-ros.git

echo ""
echo "[3/5] Cloning Slamtec SLLiDAR ROS2 driver..."
git clone https://github.com/Slamtec/sllidar_ros2.git

echo ""
echo "[4/5] Building workspace with colcon (10 mins expected)..."
cd ~/sensors_ws
colcon build --symlink-install --executor sequential

echo ""
SOURCE_LINE="source ~/sensors_ws/install/setup.bash"
if ! grep -Fxq "$SOURCE_LINE" ~/.bashrc; then
    echo "[5/5] Adding source line to ~/.bashrc..."
    echo "$SOURCE_LINE" >> ~/.bashrc
    echo "✅ Sensor path added to ~/.bashrc: $SOURCE_LINE"
else
    echo "✅ ~/.bashrc already contains the source line."
fi

echo "======================================="
echo "✅ Sensor Driver Installation complete!"
echo "======================================="

echo ""
echo "[FINAL 1/2] Sourcing the workspace in this shell..."
source ~/NavOS/install/setup.bash
source ~/sensors_ws/install/setup.bash

echo ""
echo "[FINAL 2/2] Creating udev rules for rplidar (make sure lidar is connected)..."
sudo chmod 777 /dev/ttyUSB0


echo "======================================="
echo "✅ ALL SET!"
echo "Check the README.md for Tutorial!"
echo "======================================="


