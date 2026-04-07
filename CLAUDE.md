# CLAUDE.md — Robot Arm Project

## Role

You are the engineer's right hand in code form. Thorough, attentive, proactive.
You catch what's missed, flag what's inconsistent, and surface better approaches when you see them.
The user moves fast — keep up and fill the gaps.

---

## Standing Rules — Always Follow

### Code Quality
- Professional standards. Neat, readable, no duplication, no reinventing wheels.
- Every new method gets logging. No silent code paths.
- Use the right log level: `info` for state changes, `debug` for per-frame noise, `warn` for recoverable issues, `error` for failures.
- Log messages must be meaningful: include context prefix `[COMPONENT]` and key variable values.

### Consistency
- When changing one file, check all related files for knock-on effects. Say what you checked.
- Parameter names must match across: YAML configs, node declarations, GUI controls, and this knowledge file.
- URDF component names stay snake_case everywhere.

### Never Without Asking
- Edit `robot_arm.urdf`
- Change joint limits, link geometry, or ros2_control block
- Change board origin, square size, or coordinate system
- Modify `move_group_params.yaml`

### Always After Any Change
```bash
colcon build --packages-select <pkg> --symlink-install
source ~/Desktop/Arm/install/setup.bash
```

---

## Environment
- ROS 2 Jazzy + Gazebo Harmonic 8.x on Ubuntu 24.04
- Workspace: `~/Desktop/Arm/`
- Always source before any ROS command: `source ~/Desktop/Arm/install/setup.bash`
- Always build with: `colcon build --packages-select <pkg> --symlink-install`
- New config files need `colcon build` even with `--symlink-install`

## Packages
- `robot_arm_description` — URDF, Gazebo world, controllers, camera
- `robot_arm_moveit` — MoveIt 2 config, SRDF, IK, launch
- `robot_arm_chess` — Stockfish engine, board state, arm controller, vision, GUI

## Launch
```bash
# Terminal 1 — Gazebo sim
alias arm_sim='pkill -f "gz sim" 2>/dev/null; sleep 1; rm -rf ~/.gz ~/.gazebo; ros2 launch robot_arm_description gazebo.launch.py'
arm_sim

# Terminal 2 — MoveIt (keep Gazebo running, only restart this)
ros2 launch robot_arm_moveit moveit.launch.py

# Terminal 3 — Full chess system (replaces arm_sim + moveit)
chmod +x ~/Desktop/Arm/robot_arm_chess/scripts/*.py
ros2 launch robot_arm_chess chess.launch.py

# Terminal 4 — Camera feed
ros2 run rqt_image_view rqt_image_view   # DO NOT use RViz Image display — segfaults
```

## Robot Joints
| Joint | Axis | Limits |
|---|---|---|
| base_yaw | Z | ±180° |
| shoulder_roll | Y | ±45° |
| shoulder_pitch | Y | -90°/+135° |
| elbow_pitch | Y | ±120° |
| finger_1/2/3_joint | Y | -1.2217 (closed) / 0.0 (open) |

**Finger convention: 0.0 = open, -1.2217 = closed.**
Chain: `base_link → ... → wrist_link → tool0` (MoveIt tip link)
Camera: fixed to `wrist_link` via `camera_joint`, publishes `/camera/image_raw`

---

## Hard Rules — Never Break

- Never hardcode paths in URDF — `CONTROLLERS_YAML_PATH` is a placeholder
- Never add `<plugin>` to camera sensor in URDF — native Gazebo format only
- Never remove `use_sim_time: true` from any node
- Never use RViz Image display — segfaults. Use `rqt_image_view`
- Never add `<end_effector>` to SRDF — crashes move_group
- Never pass `move_group_params.yaml` as parsed dict in launch — file path only
- Chess scripts must be `chmod +x` before build

## Critical Rules — DO NOT violate

### URDF (`robot_arm_description/urdf/robot_arm.urdf`)
- Do not edit without approval
- `CONTROLLERS_YAML_PATH` is a placeholder replaced at launch time — never hardcode a path there
- Camera sensor uses native Gazebo format with `<topic>/camera/image</topic>` — no plugin
- Fixed joints use `<preserveFixedJoint>true</preserveFixedJoint>` to prevent Gazebo merging

### Gazebo World (`robot_arm_description/worlds/arm_world.sdf`)
- Must include `gz-sim-sensors-system` plugin with `<render_engine>ogre2</render_engine>` — without it camera topics exist but publish nothing
- Chess world is at `robot_arm_chess/worlds/chess_world.sdf`

### MoveIt (`robot_arm_moveit/config/move_group_params.yaml`)
- Pass as file path string in launch, NOT as parsed dict — nested namespaces break otherwise
- `use_sim_time: true` must be set in every node — mismatch causes trajectory validation failure
- Controller manager: `moveit_simple_controller_manager/MoveItSimpleControllerManager`
- `action_ns: follow_joint_trajectory` → constructs `/arm_controller/follow_joint_trajectory`
- SRDF uses `<chain base_link="base_link" tip_link="tool0"/>` for arm group
- No `<end_effector>` block in SRDF — causes crash

### Chess Scripts
- Must be chmod +x before `colcon build` for `install(PROGRAMS ...)` to work
- `board_state_node.py` — source of truth for board state, publishes FEN
- `chess_engine_node.py` — only calculates when `board.turn == arm_color`
- `chess_arm_node.py` — uses analytical IK, sends to `/arm_controller/joint_trajectory` directly (not MoveIt)

---

## Architecture — Treat As Settled

- Arm uses **analytical IK**, not MoveIt, for chess moves
- Camera is the **only** source of game state — vision always in the loop
- GUI publishes to `/chess/gui_move` only (Gazebo teleport) — never to `/chess/human_move`
- All detection gated on `arm_idle AND _markers_stable`
- `use_sim` param gates all `gz service` calls in `chess_arm_node.py`

---

## Known Bugs — Fix In This Order

1. `square_to_xyz()` Z/XY source mismatch — all three coords from same source
2. ERROR overwritten by IDLE — guard with `if self.arm_status != 'ERROR'`
3. `_sample_perspective()` ignores board_flip
4. Engine uninitialized on bad Stockfish path — log + shutdown
5. Castling — two sequential pick/place ops
6. En-passant — remove captured pawn before main move

---

## Key Files
```
robot_arm_description/
  urdf/robot_arm.urdf              # v0.4.2 — camera, 7 joints, ros2_control block
  config/controllers.yaml          # arm_controller + gripper_controller + joint_state_broadcaster
  launch/gazebo.launch.py          # injects CONTROLLERS_YAML_PATH, spawns robot, loads controllers
  worlds/arm_world.sdf             # sensors plugin required for camera

robot_arm_moveit/
  config/move_group_params.yaml    # PRIMARY — planning pipeline + controller manager
  config/robot_arm.srdf            # groups: arm (chain), gripper (joints)
  config/kinematics.yaml           # KDL solver, arm group only
  config/joint_limits.yaml         # velocity/accel limits
  launch/moveit.launch.py          # loads all configs, starts move_group + RViz
  scripts/arm_ik.py                # analytical IK + MoveIt Python client

robot_arm_chess/
  config/board_config.yaml         # origin=(0.20,-0.175,0.02), square=0.045m
  config/chess_params.yaml         # stockfish depth=10, arm plays black
  scripts/board_state_node.py      # /chess/board_state (FEN), /chess/last_move
  scripts/chess_engine_node.py     # /chess/engine_move (UCI)
  scripts/chess_arm_node.py        # /chess/arm_move, /chess/arm_status
  scripts/chess_gui.py             # tkinter, click squares or type UCI, /chess/human_move
  worlds/chess_world.sdf           # board + 32 cylinder pieces, sensors plugin included
  launch/chess.launch.py           # full system: gazebo + controllers + chess nodes
```

## Topics
```
/joint_states                               <- joint_state_broadcaster
/clock                                      <- gz_ros_bridge (gz.msgs.Clock)
/robot_description                          <- robot_state_publisher
/arm_controller/joint_trajectory            <- send JointTrajectory to move arm
/gripper_controller/joint_trajectory        <- send JointTrajectory to move gripper
/arm_controller/follow_joint_trajectory     <- action server (MoveIt uses this)
/gripper_controller/follow_joint_trajectory <- action server
/camera/image_raw                           <- gz_ros_bridge (remapped from /camera/image)
/chess/human_move                           <- GUI publishes UCI string (e.g. "e2e4")
/chess/board_state                          <- FEN string, updated after every move
/chess/engine_move                          <- Stockfish best move UCI
/chess/arm_move                             <- confirms arm executed move
/chess/arm_status                           <- IDLE / MOVING / DONE / ERROR
/chess/game_status                          <- ONGOING / CHECK / CHECKMATE / STALEMATE

# Hardware Layer topics
/arm/joint_targets                          <- ROS → Picos (target angles, JointState)
/arm/joint_actual                           <- Picos → ROS (measured angles, JointState)
/arm/joint_drift                            <- Picos → ROS (drift warnings, String JSON)
```

## Debugging
```bash
# Controllers loaded?
ros2 control list_controllers

# Joint states publishing?
ros2 topic echo /joint_states --once

# Camera publishing?
ros2 topic hz /camera/image_raw

# Camera in Gazebo but not ROS?
gz topic -l | grep camera      # check gz side
gz topic -e -t /camera/image   # should stream data

# Chess topics alive?
ros2 topic list | grep chess

# Send test human move
ros2 topic pub --once /chess/human_move std_msgs/msg/String "data: 'e2e4'"

# MoveIt action servers up?
ros2 action list
# Expected: /arm_controller/follow_joint_trajectory
#           /gripper_controller/follow_joint_trajectory
#           /move_action
#           /execute_trajectory
```

## Known Issues
| Issue | Workaround |
|---|---|
| RViz Image display -> segfault | Use `rqt_image_view` |
| `pitch-3.1416` URDF parse warning | Cosmetic only, ignore |
| Camera rate ~12Hz not 30Hz | GPU overhead, acceptable |
| Chess scripts not found at launch | `chmod +x scripts/*.py` then rebuild |
| Ros2ControlManager doesn't connect | Use MoveItSimpleControllerManager with `action_ns: follow_joint_trajectory` |

---

## Hardware Layer (v0.5+)

### Control Architecture
```
ROS 2 (PC)
    │  joint angle targets (~50Hz)
    ▼
Pico 1 (micro-ROS) — base_yaw + shoulder
Pico 2 (micro-ROS) — elbow + wrist
    │
    ├── Reads potentiometers (onboard ADC)
    ├── Reads encoders (interrupt pins) — if fitted
    ├── Compares commanded vs actual angle
    ├── Flags drift > threshold to ROS
    └── Reports actual joint positions back to ROS
    │
    ▼
Arduino CNC Shield
    └── A4988 drivers → stepper motors (open-loop)
```

### Pico Assignment
| Pico | Joints | ADC pins used | Spare ADC |
|---|---|---|---|
| Pico 1 | base_yaw, shoulder | GP26, GP27 | GP28 |
| Pico 2 | elbow, wrist | GP26, GP27 | GP28 |

### Pico Role — Watchdog/Feedback Only
- Steppers are open-loop (A4988, no step feedback)
- Picos do NOT control motors — Arduino CNC shield does
- Picos monitor actual joint angle via pots and report to ROS
- If actual angle drifts from commanded angle beyond threshold → publish warning to `/arm/joint_drift`
- ROS decides what to do (stop, recalibrate, continue)

### A4988 Notes
- Current limit set via trim pot — set carefully, main cause of missed steps and heat
- `ENABLE` pin active low — CNC shield handles this
- Logic 5V, motor voltage separate (up to 35V)
- Microstepping up to 1/16 via MS1/MS2/MS3 pins on shield

### Pico ADC Notes
- Known noisy ADC — add 100nF cap between wiper pin and GND at the pot
- Use `AGND` and `ADC_VREF` pins, not regular GND
- Average 8–16 readings per sample in firmware
- 12-bit ADC → ~0.066° resolution over 270° pot range

---

## Adding New Features

**New ROS node:** scripts/ + chmod +x + CMakeLists.txt + chess.launch.py TimerAction + document topics above

**New config param:** board_config.yaml or chess_params.yaml + declare_parameter() + document units

**New hardware node:** follow Pico assignment table, use micro-ROS client, publish to topics above

**New tunable param:** add slider/control to `vision_calib_gui.py` — never skip this

---

## Proactive Checks — Do These When Touching Hardware Code

- Does commanded angle match what the pot is actually reading in sim/test?
- Is drift threshold appropriate for the joint's expected load?
- Is the Pico publishing at sufficient rate for ROS to use (min 50Hz for control)?
- Are stepper step counts staying in sync with commanded positions across a full move sequence?

---

## Git

```bash
# Hardware/sim changes
git checkout arm && git add -A && git commit -m "msg" && git push origin arm

# Chess changes
git checkout chess && git add -A && git commit -m "msg" && git push origin chess

# Pull arm into chess
git checkout chess && git merge arm && git push origin chess

# Stable release
git checkout main && git merge chess && git push origin main
```

### Branches
- `arm` — robot hardware only (URDF, sim, MoveIt). Switch here to modify the arm.
- `chess` — chess system on top of arm. Merge from `arm` to get arm updates.

---

## README
- Update after significant features/fixes — only when user gives permission
- User decides version number — never bump without being told
- Changelog newest-first, features before fixes

---

## Pending
- [ ] Tune chess pick/place IK coordinates against actual board positions in Gazebo
- [ ] Replace cylinder chess pieces with STL meshes
- [ ] Add castling / en-passant to chess_arm_node
- [ ] Switch chess_arm_node from analytical IK to MoveIt planning
- [ ] Vision pipeline: detect real board from `/camera/image_raw`
- [ ] Fusion 360 STL meshes for arm links (`robot_arm_description/meshes/`)
