#!/usr/bin/env bash
#
# setup_qbot_env.sh — Provision a fresh Ubuntu 24.04 (Raspberry Pi 5) for QBot.
#
# Installs ROS 2 Jazzy (ros-base, kept light for 4 GB RAM), build tools, the
# Python/OpenCV deps used by the QBot nodes, and Kinect (libfreenect) USB
# permissions. Safe to re-run (idempotent).
#
# Usage (needs sudo for apt + udev — you'll be prompted for your password):
#     bash setup_qbot_env.sh
#
set -euo pipefail

ROS_DISTRO=jazzy
PROJECT_USER="${SUDO_USER:-$USER}"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> QBot setup: ROS 2 ${ROS_DISTRO} | user '${PROJECT_USER}' | workspace ${WS_DIR}"

# 1. Locale --------------------------------------------------------------------
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Prerequisites + Universe repo --------------------------------------------
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe

# Ensure the noble-updates pocket is enabled. Some RPi images ship with only
# 'noble' + 'noble-security'; without -updates, security-patched runtime libs
# outrun their matching -dev packages and ROS install fails on broken deps.
UBUNTU_SOURCES=/etc/apt/sources.list.d/ubuntu.sources
if [ -f "${UBUNTU_SOURCES}" ] && ! grep -qE '^Suites:.*noble-updates' "${UBUNTU_SOURCES}"; then
  sudo cp "${UBUNTU_SOURCES}" "${UBUNTU_SOURCES}.bak"
  sudo sed -i 's|^Suites: noble$|Suites: noble noble-updates noble-backports|' "${UBUNTU_SOURCES}"
fi

# 3. ROS 2 apt source ----------------------------------------------------------
sudo apt update
ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F\" '{print $4}')"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${VERSION_CODENAME}")_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

# 4. ROS 2 Jazzy + build/dev tools --------------------------------------------
# full-upgrade first: ROS dev packages pin runtime libs to the patched (-updates)
# versions, so the base system must be fully upgraded or apt reports broken deps.
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y \
  ros-${ROS_DISTRO}-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential cmake git

# 5. ROS packages used by the QBot nodes --------------------------------------
sudo apt install -y \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-std-msgs \
  python3-opencv \
  python3-numpy

# 6. rosdep --------------------------------------------------------------------
sudo rosdep init 2>/dev/null || true
rosdep update || true

# 7. Kinect / libfreenect USB access ------------------------------------------
sudo apt install -y libfreenect-dev freenect python3-freenect 2>/dev/null || \
  sudo apt install -y libfreenect-dev freenect || true
# video,plugdev -> Kinect ; dialout -> Kobuki USB-serial
sudo usermod -aG video,plugdev,dialout "${PROJECT_USER}"

sudo tee /etc/udev/rules.d/51-kinect.rules >/dev/null <<'RULES'
# Xbox Kinect v1 / Kinect for Windows (Microsoft, vendor 045e) — unprivileged access
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666", GROUP="plugdev"
RULES

# The in-kernel gspca_kinect driver claims the camera and blocks libfreenect.
sudo tee /etc/modprobe.d/blacklist-kinect.conf >/dev/null <<'EOF'
# Let libfreenect (OpenKinect) own the Kinect camera instead of the kernel driver
blacklist gspca_kinect
EOF
sudo modprobe -r gspca_kinect 2>/dev/null || true

# Kobuki base: FTDI FT232 USB-serial -> stable /dev/kobuki symlink + group access
sudo tee /etc/udev/rules.d/60-kobuki.rules >/dev/null <<'RULES'
# Kobuki mobile base (Yujin Robot "iClebo Kobuki", FTDI FT232) -> /dev/kobuki.
# Match on the product string so we don't grab unrelated 0403:6001 FTDI adapters.
SUBSYSTEM=="tty", ATTRS{manufacturer}=="Yujin Robot", ATTRS{product}=="iClebo Kobuki", MODE="0666", GROUP="dialout", SYMLINK+="kobuki"
RULES
sudo udevadm control --reload-rules && sudo udevadm trigger

# 8. Auto-source ROS in the user's shell --------------------------------------
SHELL_RC="/home/${PROJECT_USER}/.bashrc"
if ! grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" "${SHELL_RC}" 2>/dev/null; then
  echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${SHELL_RC}"
fi

cat <<EOF

==> Done.
    1. Log out/in (or run: newgrp plugdev) so the video/plugdev groups take effect.
    2. Build the workspace:
         source /opt/ros/${ROS_DISTRO}/setup.bash
         cd ${WS_DIR}
         colcon build --symlink-install
         source install/setup.bash
    3. Bring it up:
         ros2 launch behavior_node qbot.launch.py
EOF
