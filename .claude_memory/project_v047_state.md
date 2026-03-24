---
name: Project state at v0.4.8
description: Current version, what's working, key architecture decisions, and remaining work
type: project
---

**Current version: v0.4.8** (branch: chess)

**What's working:**
- Full chess system launches via `ros2 launch robot_arm_chess chess.launch.py` (sim)
- Real-hardware launch via `ros2 launch robot_arm_chess chess_real.launch.py` (no Gazebo)
- `chess_arm_node.py` `use_sim` param (default True) gates all `gz service` subprocess calls
- ArUco 4-marker 16-pt LMEDS homography → accurate perspective-correct grid overlay
- Vision-driven human move detection: camera detects piece moves, publishes `/chess/human_move`
- All detection gated on `arm_idle AND _markers_stable` (ArUco centroid drift < 2px for 8 frames)
- Capture bug fixed: `cap_model != moving_model` guard in `human_move_cb`
- Post-RESET auto-snapshot: 60-frame settle + marker stability → re-enables human monitoring
- ArmTunerGUI (`vision_calib_gui.py`): 6-tab live param tuning, SetParameters to both nodes
- RECAL/RESET semantics correct: RECAL resets detection, RESET resets game state

**Key architecture:**
- bx = rank direction, by = file direction in homography board-space
- `_markers_stable` is the universal detection gate (not just arm_idle)
- `_init_piece_map()` clears dict before rebuild (stale entry bug was root of capture issues)
- `chess_vision_node.py` publishes to `/chess/human_move` (vision-driven, not GUI)
- `chess_real.launch.py`: starts `ros2_control_node` + controllers + chess nodes, `use_sim_time: false`, `use_sim: false`

**Why:** Arm plays black. Human (white) moves first. Vision monitors every frame after arm idles.

**How to apply:** When touching vision/arm chess code, check if detection gating logic is correct — both arm_idle AND _markers_stable must be True. Don't revert to arm_idle-only gating.

**Remaining work:**
- Tune IK pick/place coordinates vs actual Gazebo board positions
- Replace cylinder pieces with STL meshes
- Castling / en-passant in chess_arm_node
- Switch from analytical IK to MoveIt planning
- Fusion 360 STL meshes for arm links
