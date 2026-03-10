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

### Slider GUI controller (recommended)
```bash
# In a second terminal while Gazebo is running
ros2 run robot_arm_description arm_slider_gui.py
```
Sliders for all joints in degrees, adjustable move duration, and preset buttons.

### Move arm joints (terminal)
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

### v0.4 — All-Y-axis joints + slider GUI controller
- Changed `shoulder_roll` joint axis from X → Y (now pitches forward/back like all other joints)
- Changed `finger_2_joint` axis from `0.866 0.5 0` → `0 1 0`
- Changed `finger_3_joint` axis from `0.866 -0.5 0` → `0 1 0`
- All joints except `base_yaw` now rotate around Y axis consistently
- Added `scripts/joint_slider_controller.py` — tkinter-based slider GUI
  - Separate sliders for all 4 arm joints and 3 gripper fingers
  - Live publishing on every slider move (no need to click Send)
  - 5 presets: HOME, REACH, PICK READY, CLOSE GRIP, OPEN GRIP
  - Status bar shows last sent positions
- Added scripts to `CMakeLists.txt` install targets

### v0.4.0 — Slider GUI controller
- Added `scripts/arm_slider_gui.py` — tkinter-based joint position slider panel
- Sliders for all 4 arm joints (degrees) + 3 gripper fingers
- Adjustable move duration (0.5s–5s)
- 6 presets: Home, Reach, Pick, Wave, Grip Close, Grip Open
- Publishes to `/arm_controller/joint_trajectory` and `/gripper_controller/joint_trajectory`
- ROS spin runs in background thread so GUI stays responsive
- All joints standardized to Y axis (pitch) — `shoulder_roll`, `shoulder_pitch`, `elbow_pitch`, all fingers now `axis xyz="0 1 0"`
- `base_yaw` remains on Z axis

### v0.3.6 — Controller timing fix
- Set `use_sim_time: false` consistently across all nodes and controllers
- Added `open_loop_control: true` to arm and gripper controllers — prevents position feedback from rejecting commands when sim clock is inconsistent
- Added `allow_integration_in_goal_trajectories: true` — allows sending trajectories without velocity/acceleration fields
- Set `stopped_velocity_tolerance: 0.0` — removes strict velocity check that was causing goal rejection
- Root cause: trajectory timestamps were being compared against wrong clock source, causing immediate goal expiry and snap-back to zero

### v0.3.5 — Clock bridge fix + first successful run
- All 3 controllers loading and activating successfully in Gazebo Harmonic
- Fixed `/clock` bridge — added world-scoped clock topic and `use_sim_time` to bridge node
- `controller_manager` was running on wall clock due to missing sim clock — resolved

### v0.3.4 — Hardware plugin architecture fix (patch)
- Reverted standalone `ros2_control_node` approach — `gz_ros2_control/GazeboSimSystem` hardware plugin only exists inside Gazebo, not as a standalone ROS node
- Restored in-Gazebo `libgz_ros2_control-system.so` plugin in URDF
- Replaced `$(find ...)` xacro syntax with `CONTROLLERS_YAML_PATH` placeholder string
- Launch file now does `urdf.replace('CONTROLLERS_YAML_PATH', controllers_file)` to inject the real absolute path at launch time
- Added `GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib` env var to Gazebo process so plugin `.so` is found
- Removed standalone `controller_manager` node from launch (controller_manager is now created by the Gazebo plugin)

### v0.3.3 — controller_manager topic fix (patch)
- In ROS 2 Jazzy, `ros2_control_node` reads `robot_description` from the `/robot_description` **topic**, not a parameter
- Removed `robot_description` from `controller_manager` parameter dict
- Added `remappings=[('~/robot_description', '/robot_description')]` so it subscribes to `robot_state_publisher`'s topic
- Added `use_sim_time: True` to `controller_manager`

### v0.3.2 — Gazebo launch fixes (patch)
- Removed `$(find ...)` xacro syntax from URDF plugin tag (invalid in plain URDF, caused params-file parse crash)
- Removed `<parameters>` from URDF plugin block entirely
- Switched from in-plugin controller loading to standalone `ros2_control_node` in launch file — more reliable with Jazzy + Gazebo Harmonic 8
- `controllers_file` path now resolved in Python via `get_package_share_directory` and passed directly to `controller_manager` node
- Added `TimerAction(5s)` before loading controllers to ensure `controller_manager` is ready

### v0.3.1 — Inertia fix (patch)
- Recalculated all link inertia tensors from first principles (geometry + mass)
- Fixed `shoulder_roll_link` ixx/iyy: `0.000010` → `0.000023` (was causing Gazebo Error Code 19 invalid inertia)
- Fixed `base_link`, `upper_arm`, `forearm`, `wrist_link` minor inertia errors
- All values now computed as solid primitives: cylinder `ixx = (1/12)m(3r²+h²)`, box `ixx = (1/12)m(y²+z²)`


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
