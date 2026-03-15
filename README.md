# Robot Arm — ROS 2 Chess System

ROS 2 Jazzy · Gazebo Harmonic 8.x
3-DOF arm with 3-finger gripper — simulated chess-playing robot with vision pipeline.

---

## Packages

| Package | Purpose |
|---|---|
| `robot_arm_description` | URDF, Gazebo world, controllers, wrist camera |
| `robot_arm_moveit` | MoveIt 2 config, SRDF, IK, RViz launch |
| `robot_arm_chess` | Stockfish engine, board state, arm controller, vision, GUI |

---

## Robot Overview

```
world (fixed)
 └─ base_link                   cylinder  Ø120×60mm   0.5 kg
     └─ [base_yaw]              revolute  Z   ±180°
         └─ shoulder_roll_link  cylinder  Ø50×40mm    0.08 kg
             └─ [shoulder_roll] revolute  Y   ±45°
                 └─ upper_arm   box       40×40×200mm 0.3 kg
                     └─ [shoulder_pitch]  revolute  Y  -90° to +135°
                         └─ forearm       box  35×35×180mm  0.25 kg
                             └─ [elbow_pitch]  revolute  Y  ±120°
                                 └─ wrist_link  cylinder  Ø60×50mm  0.1 kg
                                     ├─ [finger_1/2/3_joint]  revolute  0° to -70°
                                     ├─ camera_link  (fixed, wrist-mounted)
                                     └─ tool0  (massless end-effector frame)
```

**Finger convention:** `0.0` = open · `-1.2217` = closed
**Total joints:** 9 · **Controllable DOF:** 7

---

## Dependencies

```bash
sudo apt install \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  gz-harmonic liburdfdom-tools python3-opencv python3-chess stockfish
```

---

## Build

```bash
cd ~/Desktop/Arm
colcon build --symlink-install
source install/setup.bash
```

After `--symlink-install`, edits to URDF, YAML, SDF, and launch files are **live immediately**.
Only rebuild if you add new files or change `CMakeLists.txt`.

---

## Running

### Full chess system (recommended)
```bash
# Terminal 1 — Gazebo + controllers + all chess nodes
chmod +x ~/Desktop/Arm/robot_arm_chess/scripts/*.py
ros2 launch robot_arm_chess chess.launch.py

# Terminal 2 — Camera debug feed
ros2 run rqt_image_view rqt_image_view
# Subscribe to /chess/vision/debug_image
```

### Arm simulation only
```bash
# Terminal 1
ros2 launch robot_arm_description gazebo.launch.py

# Terminal 2
ros2 launch robot_arm_moveit moveit.launch.py
```

---

## Chess System Architecture

```
/camera/image_raw
        │
        ▼
chess_vision_node  ──────────────────────────────────────────────────────
  • ArUco 4-marker homography (16-pt LMEDS overdetermined)               │
  • Gated on: arm_idle AND _markers_stable (ArUco centroid drift < 2px)  │
  • SEARCHING → WAIT_EMPTY → CAPTURING_REF → WAIT_PIECES → TRACKING      │
  • Detects human piece move → publishes /chess/human_move               │
        │                                                                 │
        ▼                                                                 │
board_state_node  (FEN tracking)                                         │
        │                                                                 │
        ▼                                                                 │
chess_engine_node  (Stockfish depth=10, plays black)                     │
        │                                                                 │
        ▼                                                                 │
chess_arm_node  ─────────────────────────────────────────────────────────
  • Analytical IK → /arm_controller/joint_trajectory
  • Teleports pieces in Gazebo to match real board state
  • Publishes /chess/arm_status (IDLE/MOVING/DONE/ERROR)
```

### Vision pipeline states

| State | Meaning | Entry condition |
|---|---|---|
| SEARCHING | Looking for ArUco markers | startup / RESET without grid |
| WAIT_EMPTY | Board found, waiting for calibration | ArUco 4-marker detected |
| CAPTURING_REF | Recording empty-board brightness reference | RECAL command |
| WAIT_PIECES | Waiting for all 32 pieces to be placed | Reference captured |
| TRACKING | Live game — detecting moves | All 32 pieces seen |

### Key topics

```
/chess/human_move          ← vision publishes detected human moves (UCI)
/chess/engine_move         ← Stockfish best move
/chess/arm_status          ← IDLE / MOVING / DONE / ERROR
/chess/board_state         ← FEN string
/chess/last_move           ← last executed UCI move
/chess/cmd                 ← command bus: RECAL, RESET, REMOVE_PIECES, STANDBY
/chess/vision/debug_image  ← annotated camera feed
```

### GUI commands

```
RECAL           — remove pieces, capture empty-board reference, return to tracking
RESET           — reset game to start position (keeps vision calibration)
REMOVE_PIECES   — teleport all pieces to graveyard
RETURN_PIECES   — return pieces to starting squares
STANDBY         — move arm to standby pose
```

---

## Tuning GUI

```bash
python3 ~/Desktop/Arm/robot_arm_chess/scripts/vision_calib_gui.py
```

Tabs: **Vision** · **Detection** · **Board Setup** · **Movement** · **Standby** · **Commands**

All parameters update live via ROS 2 `SetParameters` service — no restart needed.

---

## Debugging

```bash
# Controllers loaded?
ros2 control list_controllers

# Camera publishing?
ros2 topic hz /camera/image_raw

# Chess topics alive?
ros2 topic list | grep chess

# Send test human move
ros2 topic pub --once /chess/human_move std_msgs/msg/String "data: 'e2e4'"

# Trigger calibration
ros2 topic pub --once /chess/cmd std_msgs/msg/String "data: 'RECAL'"
```

---

## Planned / Next Steps

- [ ] Tune chess pick/place IK coordinates against actual board positions in Gazebo
- [ ] Replace cylinder chess pieces with STL meshes
- [ ] Add castling / en-passant support to chess_arm_node
- [ ] Switch chess_arm_node from analytical IK to MoveIt planning
- [ ] Fusion 360 STL meshes for arm links

---

## Changelog

### v0.4.7 — Vision-driven move detection + camera stability gate + bug fixes
- **Vision-driven human move detection**: camera now detects when a human moves a piece
  and publishes to `/chess/human_move` automatically — no GUI click needed
  - Detects simple moves (piece disappears from A, appears on B)
  - Detects captures (piece disappears from A, brightness changes on occupied B)
  - `MOVE_STABLE_FRAMES = 15` consecutive frames required before publishing
- **Camera stability gate** (`_markers_stable`): all detection functions now require
  BOTH `arm_idle` AND that ArUco marker centroid drift < 2px for 8 consecutive frames
  - Prevents sampling blurred frames while arm is settling after "IDLE" is published
  - Applied to: SEARCHING, CAPTURING_REF, WAIT_PIECES, TRACKING idle snapshot, human move monitor
  - HUD shows `CAM:STILL` / `CAM:DRIFT(n/8)` live
- **Stability timeout fallback** (`chess_vision_node.py`): if ArUco marker centroid
  drift never drops below threshold after the arm idles (Gazebo controller oscillation),
  the idle snapshot fires anyway after 30 frames (~2.5 s) so the game can never get
  permanently stuck waiting for camera stability
- **Capture bug fix** (`chess_arm_node.py`): `human_move_cb` now fetches both
  `cap_model` and `moving_model` before any teleports; added `cap_model != moving_model`
  guard to prevent the moving piece being sent to graveyard on a stale map
- **GUI/vision flow fix**: GUI now publishes to `/chess/gui_move` (Gazebo teleport only);
  vision is the sole publisher of `/chess/human_move` (game trigger); arm skips
  re-teleport for GUI moves via `_pending_gui_moves` set
- **GUI visual update fix** (`chess_gui.py`): clicking a piece now pushes the move
  onto the local board and redraws the canvas immediately; previously the board
  only updated after `/chess/board_state` arrived from vision (~1+ s later)
- **Engine reset bug fix** (`chess_engine_node.py`): added `/chess/cmd` subscription;
  RESET now resets `game_active = True` so a new game can start after checkmate/stalemate
  (previously `game_active` stayed False and the engine silently ignored all moves)
- **Double Stockfish analysis removed** (`chess_engine_node.py`): removed the redundant
  `engine.analyse()` call after `engine.play()` — roughly halves engine response time
- **Post-RESET auto-snapshot**: after RESET, pieces teleport back via Popen (no arm
  motion → no `_arm_just_idled`); 60-frame settle counter + marker stability check
  auto-snapshots the board and re-enables human move monitoring
- Added comprehensive logging to `_update_marker_stability`, `_monitor_human_move`,
  `human_move_cb` — all key decision branches now log at appropriate levels

### v0.4.6 — Board detection + full GUI tuning
- ArUco 4-marker homography (16-pt LMEDS overdetermined) — accurate perspective grid
- Hough fallback commented out (kept for reference); ArUco is primary detector
- Full `ArmTunerGUI` with 6 tabs: Vision, Detection, Board Setup, Movement, Standby, Commands
- Live `SetParameters` to both vision and arm nodes from GUI
- bx/by coordinate swap fix — board squares were 90° off; rank → bx, file → by
- RECAL semantic fix: mid-game RECAL skips WAIT_PIECES, goes straight to TRACKING
- RESET semantic fix: keeps grid + empty_ref, resets only game state
- `_init_piece_map()` clears dict before rebuild — fixed stale-entry capture bug

### v0.4.5 — Bug fixes, standby position, board detection work
- Fixed right standby position
- Working ON board detection pipeline
- Added live param callback for standby pose joints in chess_arm_node

### v0.4.4 — Working chess GUI
- Tkinter chess GUI with clickable board
- Human moves via GUI click or UCI text input
- Bug fixes across chess nodes

### v0.4.3 — Chess system started
- `robot_arm_chess` package: Stockfish engine node, board state node, arm chess controller
- Analytical IK for chess pick/place
- Chess world SDF with 32 cylinder pieces and 4 ArUco board markers
- `chess.launch.py` — full system in one launch

### v0.4.2 — Camera integration
- Wrist camera added to URDF (`camera_link`, `camera_joint` fixed to `wrist_link`)
- Native Gazebo camera sensor format with `gz-sim-sensors-system` plugin
- `/camera/image_raw` bridged via `gz_ros_bridge`
- `rqt_image_view` confirmed working (RViz Image display causes segfault — do not use)

### v0.4.0 — All-Y-axis joints + slider GUI
- All joints except `base_yaw` now rotate around Y axis consistently
- Added `arm_slider_gui.py` — tkinter joint position sliders, 6 presets
- MoveIt 2 integration with KDL IK solver

### v0.3.x — Controller + simulation fixes
- v0.3.6: open-loop control, timing fixes
- v0.3.5: clock bridge fix — first successful Gazebo run
- v0.3.4: hardware plugin architecture fix (in-Gazebo `gz_ros2_control`)
- v0.3.1: inertia tensor fixes (all links from first principles)

### v0.2 — Joint additions
- Added `shoulder_roll` joint (±45°)
- Fixed finger curl axes, joint limits

### v0.1 — Initial URDF
- 3-DOF arm: `base_yaw`, `shoulder_pitch`, `elbow_pitch`
- 3-finger gripper, `tool0` end-effector frame
- RViz display launch
