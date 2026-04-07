---
name: Project state at v0.4.8
description: Current version, repo structure, what's working, key architecture decisions, and remaining work
type: project
---

**Current version: v0.4.8**
**Repo: single `main` branch** (consolidated 2026-04-07 — all branches merged)
**Archive tags:** `archive/chess`, `archive/ARM_SIM`, `archive/Design`

---

## Repo Structure

```
Chess_ARM/
├── robot_arm_description/   ← CORE: URDF, Gazebo world, controllers
├── robot_arm_moveit/        ← CORE: MoveIt, IK, motion planning
├── robot_arm_chess/         ← ADDON: Stockfish, vision, GUI, board nodes
└── hardware/                ← HARDWARE DESIGN (not ROS, not built by colcon)
    ├── arm_calculator/      FastAPI torque/geometry tool
    ├── stepper_test/        Arduino CNC Shield sketches
    ├── drawings/            SVG arm geometry
    └── params/              Fusion 360 exported params
```

New addons → `robot_arm_<addon>/` at repo root, same level as `robot_arm_chess/`.

---

## What's Working (v0.4.8)

- Full chess system: `ros2 launch robot_arm_chess chess.launch.py` (sim)
- Real-hardware: `ros2 launch robot_arm_chess chess_real.launch.py` (no Gazebo)
- `use_sim` param gates all `gz service` calls in `chess_arm_node.py`
- ArUco 4-marker 16-pt LMEDS homography → perspective-correct grid overlay
- Vision-driven human move detection → publishes `/chess/human_move`
- Detection gated on `arm_idle AND _markers_stable` (ArUco centroid drift < 2px for 8 frames)
- ArmTunerGUI (`vision_calib_gui.py`): 6-tab live param tuning, SetParameters to both nodes
- RECAL resets detection; RESET resets game state

## Key Architecture

- bx = rank direction, by = file direction in homography board-space
- `_markers_stable` is the universal detection gate — never revert to arm_idle-only
- `_init_piece_map()` clears dict before rebuild (stale entry bug was root of capture issues)
- `chess_vision_node.py` publishes to `/chess/human_move` (vision, not GUI)
- `chess_real.launch.py`: `ros2_control_node` + controllers + chess nodes, `use_sim_time: false`, `use_sim: false`
- Arm plays black. Human (white) moves first.

---

## Pending Work

- [ ] Tune IK pick/place coordinates vs actual Gazebo board positions
- [ ] Replace cylinder pieces with STL meshes
- [ ] Castling — two sequential pick/place ops in chess_arm_node
- [ ] En-passant — remove captured pawn before main move
- [ ] Switch chess_arm_node from analytical IK to MoveIt planning
- [ ] Fusion 360 STL meshes for arm links (`robot_arm_description/meshes/`)
- [ ] Hardware layer: micro-ROS Pico nodes for joint feedback
- [ ] Vision pipeline: detect real board from `/camera/image_raw` on physical hardware

## Completed

- [x] Vision pipeline: camera detects human moves via ArUco+homography (v0.4.6/0.4.7)
- [x] Vision-driven move detection — arm waits for camera stability (v0.4.7)
- [x] Sim / real hardware separation — `use_sim` param + `chess_real.launch.py` (v0.4.8)
- [x] Repo consolidated into single main branch (2026-04-07)
