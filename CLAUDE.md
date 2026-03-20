# CLAUDE.md

## Standing Instructions — Always Follow These

you are a profissional developer, working with high level standards make the code neat and readable. keeping everything orginised and avoiding code duplications and reiventing wheels

### README
- Update `README.md` (root) after any significant feature, fix, or refactor — but only when the user gives permission or asks for it
- The user decides the version number — never bump the version without being told which version we are on
- Keep the Changelog accurate and complete — every change session gets its own entry
- **Keep README and changelog clearly ordered and structured**: use consistent heading hierarchy, group related items under sub-bullets, lead each bullet with a bold label (e.g. `**Feature name**`), and order changelog entries newest-first. Within a version entry, order items: new features first, then improvements, then bug fixes.

### Logging
- Always add `get_logger()` calls to new methods and any code paths that make decisions, change state, or can fail
- Use the right level: `info` for state changes and key actions, `debug` for per-frame noise, `warn` for unexpected-but-recoverable, `error` for failures
- Never leave a new method completely without logging — even a single info on entry/exit is sufficient
- When asked to add a feature, check surrounding code for missing logs and add them too
- **Keep log messages clearly ordered and structured**: include a state/context prefix (e.g. `[TRACKING]`, `[MONITOR]`, `[CAM]`) and key variable values so logs read as a clear narrative — avoid vague messages like "done" or "called"

### vision_calib_gui.py (ArmTunerGUI)
- NEVER remove a tab, slider, button, or parameter from the tuning GUI without explicit permission
- When adding new tunable parameters to any node, add the corresponding slider/control to the GUI
- Keep all existing functionality intact when refactoring the GUI

### General
- When making changes across multiple files, check all related files for consistency
- After autocompact: re-read CLAUDE.md at session start — these instructions always apply

## Environment
- ROS 2 Jazzy + Gazebo Harmonic 8.x on Ubuntu 24.04
- Workspace: `~/Desktop/Arm/`
- Always source before any ROS command: `source ~/Desktop/Arm/install/setup.bash`
- Always build with: `colcon build --packages-select <pkg> --symlink-install`
- New config files need `colcon build` even with `--symlink-install`

## Packages
- `robot_arm_description` — URDF, Gazebo world, controllers, camera
- `robot_arm_moveit` — MoveIt 2 config, SRDF, IK, launch
- `robot_arm_chess` — Stockfish engine, board state, arm chess controller, GUI

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

## Key Files
```
robot_arm_description/
  urdf/robot_arm.urdf              # v0.4.2 — camera, 7 joints, ros2_control block (last changed v0.4.2)
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
  scripts/chess_arm_node.py        # /chess/arm_move, /chess/arm_status  (v0.4.7 — capture fix, logging)
  scripts/chess_vision_node.py     # ArUco homography, piece tracking, human move detection  (v0.4.7)
  scripts/chess_gui.py             # tkinter, click squares or type UCI, /chess/human_move
  scripts/vision_calib_gui.py      # ArmTunerGUI — 6-tab live param tuning GUI
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
/chess/human_move                           <- vision OR GUI publishes UCI string (e.g. "e2e4")
/chess/vision/white_squares                 <- JSON list of squares with white pieces
/chess/vision/debug_image                   <- annotated camera feed (subscribe in rqt_image_view)
/chess/board_state                          <- FEN string, updated after every move
/chess/engine_move                          <- Stockfish best move UCI
/chess/arm_move                             <- confirms arm executed move
/chess/arm_status                           <- IDLE / MOVING / DONE / ERROR
/chess/game_status                          <- ONGOING / CHECK / CHECKMATE / STALEMATE
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

## Git Branches
- `arm` — robot hardware only (URDF, sim, MoveIt). Switch here to modify the arm.
- `chess` — chess system on top of arm. Merge from `arm` to get arm updates.

## Pending
- [ ] Tune chess pick/place IK coordinates against actual board positions in Gazebo
- [ ] Replace cylinder chess pieces with STL meshes
- [ ] Add castling / en-passant to chess_arm_node
- [ ] Switch chess_arm_node from analytical IK to MoveIt planning
- [ ] Fusion 360 STL meshes for arm links (`robot_arm_description/meshes/`)
- [x] Vision pipeline: camera detects human moves via ArUco+homography (v0.4.6/0.4.7)
- [x] Vision-driven move detection — arm waits for camera stability before sampling (v0.4.7)

## Chess Scripts — Key Behaviours (v0.4.7)
- `chess_vision_node.py` — all detection gated on `arm_idle AND _markers_stable`
  (ArUco centroid drift < 2px for 8 frames = camera truly still)
- `chess_vision_node.py` — publishes `/chess/human_move` automatically when stable piece
  change detected; simple move (1 gone + 1 appeared) or capture (1 gone + brightness change)
- `chess_arm_node.py` — `human_move_cb` fetches `cap_model` and `moving_model` before
  any teleport; `cap_model != moving_model` guard prevents double-graveyard bug
- `_init_piece_map()` clears dict before rebuild — prevents stale entries across game resets
