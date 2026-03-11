# CLAUDE.md — Robot Arm Chess Project

This file gives AI assistants full context on this ROS 2 robotic arm simulation project.
Read this before making any changes.

---

## Project Overview

A 3-DOF robotic arm simulation in ROS 2 Jazzy + Gazebo Harmonic that plays chess using
Stockfish. The arm physically picks and places pieces on a simulated chess board.

**Workspace:** `~/Desktop/Arm/` (git repo)
**Active branches:**
- `arm` — robot hardware, URDF, simulation, MoveIt 2
- `chess` — chess engine, board state, arm chess controller, GUI

**Build:**
```bash
cd ~/Desktop/Arm
colcon build --symlink-install           # full build
colcon build --packages-select <pkg> --symlink-install  # single package
source install/setup.bash
```

---

## Packages

### robot_arm_description
Robot hardware description, Gazebo simulation, controllers.

```
urdf/robot_arm.urdf          # URDF v0.4.2 — DO NOT edit without approval
config/controllers.yaml      # ros2_control controller config
launch/gazebo.launch.py      # Main sim launch (use arm_sim alias)
worlds/arm_world.sdf         # Gazebo world with sensors system plugin
scripts/arm_slider_gui.py    # tkinter joint slider GUI
```

**Launch alias:**
```bash
alias arm_sim='pkill -f "gz sim" 2>/dev/null; sleep 1; rm -rf ~/.gz ~/.gazebo; ros2 launch robot_arm_description gazebo.launch.py'
```

### robot_arm_moveit
MoveIt 2 motion planning configuration.

```
config/move_group_params.yaml   # PRIMARY config — all move_group params
config/robot_arm.srdf           # Planning groups, named poses, collisions
config/kinematics.yaml          # KDL IK solver
config/joint_limits.yaml        # Velocity/acceleration limits
launch/moveit.launch.py         # Starts move_group + RViz
scripts/arm_ik.py               # Analytical IK + MoveIt Python client
```

**Launch:**
```bash
ros2 launch robot_arm_moveit moveit.launch.py
```

### robot_arm_chess
Chess engine integration and arm chess controller.

```
scripts/board_state_node.py    # Tracks piece positions, publishes FEN
scripts/chess_engine_node.py   # Stockfish integration, publishes best move
scripts/chess_arm_node.py      # Converts moves to pick/place trajectories
scripts/chess_gui.py           # tkinter GUI for human player
config/board_config.yaml       # Board geometry (origin, square size, heights)
config/chess_params.yaml       # Engine depth, skill level, arm speed
worlds/chess_world.sdf         # Gazebo world with chess board + 32 pieces
launch/chess.launch.py         # Full chess system launch
```

**Launch:**
```bash
ros2 launch robot_arm_chess chess.launch.py
```

**Note:** Scripts must be executable before building:
```bash
chmod +x ~/Desktop/Arm/robot_arm_chess/scripts/*.py
```

---

## Robot Structure

```
world (fixed)
 └─ base_link           cylinder Ø120×60mm  0.5kg
     └─ [base_yaw]      revolute Z  ±180°
         └─ shoulder_roll_link  cylinder Ø50×40mm  0.08kg
             └─ [shoulder_roll] revolute Y  ±45°
                 └─ upper_arm   box 40×40×200mm  0.3kg
                     └─ [shoulder_pitch] revolute Y  -90°/+135°
                         └─ forearm  box 35×35×180mm  0.25kg
                             └─ [elbow_pitch] revolute Y  ±120°
                                 └─ wrist_link  cylinder Ø60×50mm  0.1kg
                                     ├─ [finger_1_joint] revolute Y  -1.2217/0.0
                                     ├─ [finger_2_joint] revolute Y  -1.2217/0.0
                                     ├─ [finger_3_joint] revolute Y  -1.2217/0.0
                                     ├─ [fixed] → tool0
                                     └─ [fixed] → camera_link (wrist camera)
```

**Finger convention:** `0.0` = open, `-1.2217` = fully closed.

---

## Key Configuration Values

### controllers.yaml
- `arm_controller` — JointTrajectoryController for `[base_yaw, shoulder_roll, shoulder_pitch, elbow_pitch]`
- `gripper_controller` — JointTrajectoryController for `[finger_1_joint, finger_2_joint, finger_3_joint]`
- `update_rate: 100`, `use_sim_time: true`

### move_group_params.yaml
- `moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager`
- `action_ns: follow_joint_trajectory` — constructs `/arm_controller/follow_joint_trajectory`
- `use_sim_time: true` everywhere

### board_config.yaml (chess)
- Board origin (a1 center): `x=0.20, y=-0.175, z=0.02`
- Square size: `0.045m`
- Arm plays as: `black`
- Grasp height: `0.04m`, lift height: `0.20m`

---

## ROS 2 Topic Map

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/joint_states` | JointState | joint_state_broadcaster | MoveIt, RSP |
| `/clock` | Clock | gz_ros_bridge | all nodes |
| `/robot_description` | String | robot_state_publisher | Gazebo, MoveIt |
| `/arm_controller/joint_trajectory` | JointTrajectory | chess_arm_node | arm_controller |
| `/gripper_controller/joint_trajectory` | JointTrajectory | chess_arm_node | gripper_controller |
| `/camera/image_raw` | Image | gz_ros_bridge | rqt_image_view |
| `/chess/board_state` | String (FEN) | board_state_node | engine, arm |
| `/chess/human_move` | String (UCI) | chess_gui | board_state_node |
| `/chess/engine_move` | String (UCI) | chess_engine_node | chess_arm_node |
| `/chess/arm_move` | String (UCI) | chess_arm_node | board_state_node |
| `/chess/arm_status` | String | chess_arm_node | chess_gui |
| `/chess/game_status` | String | board_state_node | chess_gui, engine |

---

## Gazebo Harmonic Notes (Critical)

- **Sensors require `gz-sim-sensors-system` plugin** in the world SDF with `<render_engine>ogre2</render_engine>`. Without it, camera topics exist but publish nothing.
- **Camera sensor defined in URDF** using `<topic>/camera/image</topic>` — no plugin needed.
- **Bridge** maps `/camera/image` (Gazebo) → `/camera/image_raw` (ROS 2).
- **Fixed joints** are merged by default — use `<preserveFixedJoint>true</preserveFixedJoint>` in `<gazebo reference="joint_name">` to keep them separate.
- **URDF→SDF sensor plugins** (`libgz_ros2_camera-system.so`) do NOT exist in this setup. Use native Gazebo sensors + bridge instead.
- **Camera view:** use `rqt_image_view` — RViz Image display causes segfault on this machine.

---

## MoveIt 2 Notes (Critical)

- `move_group_params.yaml` must be passed as a **file path string** in the launch file, not as a parsed dict. New files need `colcon build` even with `--symlink-install`.
- `action_ns: follow_joint_trajectory` constructs the full action path as `{controller_name}/follow_joint_trajectory`.
- `use_sim_time: true` must be set in ALL nodes — mismatch causes trajectory validation failure.
- Planning group `arm` uses `<chain base_link="base_link" tip_link="tool0"/>` for KDL IK.
- Gripper group uses joint list (not chain) — KDL can't solve IK for branching topologies.
- SRDF `<end_effector>` block removed — caused `tool0 not in group` crash.

---

## Common Commands

```bash
# Check all controllers are active
ros2 control list_controllers

# Check joint states
ros2 topic echo /joint_states --once

# Check camera
ros2 topic hz /camera/image_raw
ros2 run rqt_image_view rqt_image_view

# Check chess topics
ros2 topic list | grep chess

# Send a human move manually (UCI format)
ros2 topic pub --once /chess/human_move std_msgs/msg/String "data: 'e2e4'"

# Trigger engine move manually
ros2 topic pub --once /chess/board_state std_msgs/msg/String "data: '<FEN string>'"

# Check Gazebo topics
gz topic -l | grep camera
gz topic -l | grep chess
```

---

## Known Issues

| Issue | Status | Notes |
|---|---|---|
| RViz Image display segfault | Known bug | Use rqt_image_view instead |
| Camera rate ~12Hz (vs 30Hz target) | Acceptable | GPU rendering overhead |
| `pitch-3.1416` URDF limit parse warning | Cosmetic | Joint still works correctly |
| Chess piece models are cylinders | v0.1 placeholder | Replace with STL meshes later |

---

## Version History

| Version | Branch | Description |
|---|---|---|
| v0.1–v0.3.6 | arm | URDF, Gazebo, controllers |
| v0.4.0 | arm | arm_slider_gui, Y-axis joints, finger limits |
| v0.4.1 | arm | shoulder_roll fix (reverted) |
| v0.4.2 | arm | Wrist camera, MoveIt 2 working, execute fixed |
| v0.1.0 | chess | Chess package: Stockfish, board state, arm controller, GUI, chess world |

---

## Roadmap

- [ ] Fix chess launch (script permissions / install issue)
- [ ] Tune pick/place heights and IK for chess board coordinates
- [ ] Replace cylinder piece models with proper STL meshes
- [ ] Add castling and en-passant handling to chess_arm_node
- [ ] Vision pipeline: camera detects real board pieces (production)
- [ ] Replace analytical IK with MoveIt 2 planning in chess_arm_node
- [ ] Fusion 360 mesh integration for arm links
- [ ] Dual-arm support (two arms, one per side)
