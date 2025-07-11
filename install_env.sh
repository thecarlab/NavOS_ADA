#!/usr/bin/env bash

set -e

echo "=================================================="
echo "ROS 2 Humble Environment Installation Script"
echo "=================================================="

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (e.g. sudo ./install_env.sh)"
  exit 1
fi

echo ""
echo "[1/8] Checking current locale..."
locale

echo ""
echo "[1/8] Checking if current locale supports UTF-8..."
if locale | grep -qi 'utf-8'; then
  echo "✅ Locale already supports UTF-8:"
  locale
else
  echo "⚠️  Locale does not support UTF-8. Installing and configuring locales..."
  apt update
  apt install -y locales

  locale-gen en_US en_US.UTF-8
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

  export LANG=en_US.UTF-8
  echo "✅ Locale configured:"
  locale
fi

echo ""
echo "[2/8] Enabling Ubuntu Universe repository..."
apt install -y software-properties-common
add-apt-repository universe

echo ""
echo "[3/8] Installing curl..."
apt update
apt install -y curl

echo ""
echo "[4/8] Adding ROS 2 apt repository via ros-apt-source..."
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
OS_CODENAME=$( . /etc/os-release && echo $VERSION_CODENAME )

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${OS_CODENAME}_all.deb"

dpkg -i /tmp/ros2-apt-source.deb

echo ""
echo "[5/8] Updating and upgrading system packages..."
apt update
apt upgrade -y

echo ""
echo "⚠️  NOTE: Ensuring systemd and udev are up to date is important on Ubuntu 22.04!"
echo "See: https://github.com/ros2/ros2/issues/1272"

echo ""
echo "[6/8] Installing ROS 2 Humble desktop metapackage..."
apt install -y ros-humble-desktop

echo ""
echo "[7/8] Installing ROS 2 Humble development tools..."
apt install ros-dev-tools

echo ""
echo "[8/8] Adding ROS 2 environment sourcing to .bashrc (for user $SUDO_USER)..."
ROS_SETUP_LINE="source /opt/ros/humble/setup.bash"
BASHRC="/home/$SUDO_USER/.bashrc"

if ! grep -Eq 'source .*/opt/ros/humble/setup' "$BASHRC"; then
  echo "$ROS_SETUP_LINE" >> "$BASHRC"
  echo "✅ Added ROS environment sourcing to $BASHRC"
else
  echo "✅ ROS sourcing already present in $BASHRC"
fi

echo ""
echo "Source ROS2  for following installation"
source /opt/ros/humble/setup.bash

echo ""
echo "=================================================="
echo "✅ ROS 2 Humble installation complete!"
echo "=================================================="

echo ""
echo "=================================================="
echo "NavOS Env Install Script"
echo "=================================================="


echo ""
echo "[1/5] Installing ackermann msgs"
apt install -y ros-humble-ackermann-msgs

echo ""
echo "[2/5] Installing PCL lib"
apt install -y ros-humble-pcl-ros

echo ""
echo "[3/5] Installing g2o pre-request lib"
apt install -y libeigen3-dev libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5

echo ""
echo "[4/5] Installing g2o lib"
apt install -y ros-humble-libg2o

echo ""
echo "[5/5] Installing realsense dependencies"
apt install -y ros-humble-diagnostic-updater

echo ""
echo "=================================================="
echo "✅ NavOS Env installation complete!"
echo "=================================================="

