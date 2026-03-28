# Installation Requirements

## Platform
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic 8.x

---

## 1. ROS 2 Jazzy (Full Desktop)

```bash
# Follow official ROS 2 Jazzy install guide:
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt install ros-jazzy-desktop
```

---

## 2. ROS 2 Packages

```bash
sudo apt install \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro \
  ros-jazzy-moveit \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-moveit-ros-planning-interface \
  ros-jazzy-moveit-kinematics \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-ros-visualization \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-rqt-image-view
```

---

## 3. Python Packages

```bash
pip install chess stockfish opencv-python numpy
```

---

## 4. Stockfish Engine (system binary)

```bash
sudo apt install stockfish
```

---

## 5. Build the Workspace

```bash
cd ~/Desktop/Arm
chmod +x robot_arm_chess/scripts/*.py
colcon build --symlink-install
source install/setup.bash
```

> **Tip:** Use `rosdep` to auto-resolve ROS deps:
> ```bash
> rosdep install --from-paths src --ignore-src -r -y
> ```
> Note: `chess`, `stockfish`, and `opencv-python` pip packages are not covered by rosdep — install them manually (step 3).

---

## Launch

```bash
# Simulation (Gazebo + chess system)
ros2 launch robot_arm_chess chess.launch.py

# Real hardware
ros2 launch robot_arm_chess chess_real.launch.py
```
