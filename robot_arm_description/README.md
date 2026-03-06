# robot_arm_description

ROS 2 Jazzy · Gazebo Harmonic 8.x  
3-DOF robotic arm with 3-finger gripper — URDF simulation package.

---

## Package Structure

```
robot_arm_description/
├── urdf/
│   └── robot_arm.urdf          # Robot description (links, joints, gazebo tags, ros2_control)
├── config/
│   └── controllers.yaml        # ros2_control: arm_controller + gripper_controller
├── launch/
│   ├── display.launch.py       # RViz only (no physics)
│   └── gazebo.launch.py        # Full Gazebo Harmonic simulation
├── worlds/
│   └── arm_world.sdf           # Ground plane + target box
├── meshes/                     # Empty — drop Fusion 360 STL exports here
├── rviz/
│   └── robot_arm.rviz          # Pre-configured RViz layout
├── CMakeLists.txt
└── package.xml
```

---

## Robot Overview

```
world (fixed)
 └─ base_link                   cylinder  Ø120×60mm   0.5 kg
     └─ [base_yaw]              revolute  Z   ±180°
         └─ shoulder_roll_link  cylinder  Ø50×40mm    0.08 kg
             └─ [shoulder_roll] revolute  X   ±45°
                 └─ upper_arm   box       40×40×200mm 0.3 kg
                     └─ [shoulder_pitch]  revolute  Y  -90° to +135°
                         └─ forearm       box  35×35×180mm  0.25 kg
                             └─ [elbow_pitch]  revolute  Y  ±120°
                                 └─ wrist_link  cylinder  Ø60×50mm  0.1 kg
                                     ├─ [finger_1_joint]  revolute  0° to -70°
                                     ├─ [finger_2_joint]  revolute  0° to -70°
                                     ├─ [finger_3_joint]  revolute  0° to -70°
                                     └─ tool0  (massless end-effector frame)
```

**Total links:** 10 · **Total joints:** 9 · **Controllable DOF:** 7

---

## Dependencies

```bash
sudo apt install \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  gz-harmonic \
  liburdfdom-tools
```

---

## Build

```bash
# First time — use symlink-install so edits take effect without rebuilding
cd ~/ros2_ws
colcon build --packages-select robot_arm_description --symlink-install
source install/setup.bash
```

After `--symlink-install`, edits to URDF, YAML, SDF, and launch files are **live immediately**.  
Only rebuild if you add new files or change `CMakeLists.txt`.

---

## Running

### RViz (visual check, no physics)
```bash
ros2 launch robot_arm_description display.launch.py
```
Use the **Joint State Publisher GUI** sliders to move all joints.

### Gazebo Harmonic (full physics simulation)
```bash
ros2 launch robot_arm_description gazebo.launch.py
```

### Validate URDF structure
```bash
check_urdf ~/ros2_ws/src/robot_arm_description/urdf/robot_arm.urdf
```

---

## Sending Commands

### Move arm joints (radians)
```bash
ros2 topic pub /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [base_yaw, shoulder_roll, shoulder_pitch, elbow_pitch],
    points: [{positions: [0.5, 0.2, 0.8, -0.5], time_from_start: {sec: 2}}]
  }' --once
```

### Close gripper
```bash
ros2 topic pub /gripper_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [finger_1_joint, finger_2_joint, finger_3_joint],
    points: [{positions: [-1.0, -1.0, -1.0], time_from_start: {sec: 1}}]
  }' --once
```

### Open gripper
```bash
ros2 topic pub /gripper_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [finger_1_joint, finger_2_joint, finger_3_joint],
    points: [{positions: [0.0, 0.0, 0.0], time_from_start: {sec: 1}}]
  }' --once
```

### Check active controllers
```bash
ros2 control list_controllers
```

### List all ROS topics
```bash
ros2 topic list
```

---

## Swapping in Fusion 360 Meshes

1. In Fusion 360: **File → Export → STL**, one body per link
2. Save each file to `meshes/` — e.g. `meshes/base_link.stl`
3. In `robot_arm.urdf`, replace each `<geometry><box .../>` or `<cylinder .../>` with:
   ```xml
   <geometry>
     <mesh filename="package://robot_arm_description/meshes/base_link.stl"/>
   </geometry>
   ```
4. Repeat for both `<visual>` and `<collision>` blocks per link
5. No rebuild needed (symlink-install)

---

## Planned / Next Steps

- [ ] Add wrist camera link + Gazebo camera plugin
- [ ] Add force/torque sensor at wrist
- [ ] MoveIt 2 integration for IK / path planning
- [ ] Replace box geometry with Fusion 360 meshes
- [ ] Pick-and-place demo script

---

## Changelog

### v0.3 — Gazebo Harmonic integration
- Added `<ros2_control>` hardware interface block to URDF
- Added `<gazebo>` material and friction tags to all links
- Added `gz_ros2_control` plugin to URDF
- Added `config/controllers.yaml` with `arm_controller` and `gripper_controller`
- Added `worlds/arm_world.sdf` with ground plane, lighting, and target box
- Added `launch/gazebo.launch.py` with controller load chain
- Updated `CMakeLists.txt` to install `config/` and `worlds/`

### v0.2 — Joint fixes
- Added `shoulder_roll` joint (±45°, X axis) between `base_yaw` and `shoulder_pitch`
- Added `shoulder_roll_link` hub link
- Fixed finger curl axes — each finger now rotates inward toward wrist center
  - `finger_1`: axis `0 1 0`
  - `finger_2`: axis `0.866 0.5 0`
  - `finger_3`: axis `0.866 -0.5 0`
- Changed finger joint limits to `lower=-1.2217, upper=0.0` (negative = closed)

### v0.1 — Initial URDF
- 3-DOF arm: `base_yaw` (Z ±180°), `shoulder_pitch` (Y -90°/+135°), `elbow_pitch` (Y ±120°)
- 3-finger gripper spaced 120° apart on wrist
- `tool0` end-effector reference frame
- RViz display launch file with `joint_state_publisher_gui`
- Browser-based 3D visualizer (Three.js, no ROS required)
