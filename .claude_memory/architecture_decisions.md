---
name: Architecture decisions — treat as settled
description: Design choices that are locked in. Don't change without discussion. Includes adding-new-features guide.
type: project
---

## Settled Decisions

- Arm uses **analytical IK**, not MoveIt, for chess moves (speed)
- Camera is the **only** source of game state — vision always in the loop
- GUI publishes to `/chess/gui_move` only (Gazebo teleport) — never to `/chess/human_move`
- All detection gated on `arm_idle AND _markers_stable`
- `use_sim` param gates all `gz service` calls in `chess_arm_node.py`
- `chess_real.launch.py` for hardware: no Gazebo, no bridge, `use_sim_time: false`

**Why:** These decisions were made to ensure the camera path is always exercised (not bypassed by GUI shortcuts), and to keep sim/real separation clean.

**How to apply:** Before changing any of these, flag it as an architecture discussion. Don't silently revert to arm_idle-only gating or allow GUI to publish game moves directly.

---

## Adding New Features

**New ROS node:**
`scripts/<name>.py` + `chmod +x` + entry in `CMakeLists.txt` + `TimerAction` in `chess.launch.py` + document topics in CLAUDE.md

**New config param:**
Add to `board_config.yaml` or `chess_params.yaml` + `declare_parameter()` in node + document units in CLAUDE.md

**New hardware node:**
Follow Pico assignment table (see hardware_knowledge.md), use micro-ROS client, publish to `/arm/` topics

**New tunable param:**
Add slider/control to `vision_calib_gui.py` — never skip this step

**New addon package:**
Create `robot_arm_<addon>/` at repo root, same level as `robot_arm_chess/`. Self-contained — never modify core packages (`robot_arm_description`, `robot_arm_moveit`) for addon-specific logic.

---

## Known Bugs — Fix In This Order

See `known_bugs.md` for full detail. Priority order:

1. `square_to_xyz()` Z/XY source mismatch — coords from different pipelines, unstable grasp
2. ERROR overwritten by IDLE — guard with `if self.arm_status != 'ERROR'`
3. `_sample_perspective()` ignores `board_flip` — wrong tile positions when flipped
4. Engine uninitialized on bad Stockfish path — log + shutdown cleanly
5. Castling — two sequential pick/place ops
6. En-passant — remove captured pawn before main move
