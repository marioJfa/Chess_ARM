---
name: Known bugs and logic issues
description: Running log of confirmed bugs, logic errors, and code quality issues found in the chess robot system
type: project
---

Last updated: 2026-03-24

## Status key
- 🔴 Open — not yet fixed
- 🟡 Partial — workaround in place, not properly fixed
- 🟢 Fixed — resolved and verified

---

## chess_arm_node.py

**🟢 `_return_pieces_to_board()` missing `flip=True`**
- Line 274 called `square_to_xyz(sq)` without `flip=True`, placing white pieces on the wrong side
- `_reset_all_pieces()` correctly passes `flip=True` — these were inconsistent
- Fixed: added `flip=True` to match reset behaviour

**🟢 `_calibrated_coords_cb` INFO log spam**
- Logged INFO on every calibrator publish, flooding terminal over long sessions
- Fixed: only logs when square count changes (first receipt or update)

**🟢 `square_to_xyz` silently ignores `flip` when calibrated coords exist**
- No comment explaining why — confusing for future readers
- Fixed: added inline comment "flip is n/a — calibrated coords are absolute world positions"

**🔴 ERROR status overwritten by IDLE in finally block**
- `execute_move()` publishes ERROR on failure, then the finally block publishes IDLE — ERROR is never seen
- Consequence: human player sees IDLE after a failed arm move and may assume it is safe to continue
- File: chess_arm_node.py ~line 584

**🔴 Capture detection relies on potentially stale `_piece_map`**
- `cap_model = self._piece_map.get(to_name)` may be out of sync if a `human_move_cb` is queued but unprocessed
- Consequence: arm may skip removing an opponent piece, causing a physical collision
- File: chess_arm_node.py ~line 547

**🔴 Z-coordinate source mismatch in pick**
- `square_to_xyz()` returns calibrated Z when calibrated coords exist, but X/Y may come from vision centroids via `_wait_for_centroid()`
- These are from different pipelines and may not be consistent
- Consequence: arm descends to wrong Z for the actual XY position, unstable grasp
- File: chess_arm_node.py ~lines 437–441

**🔴 Castling not handled**
- `execute_move()` only moves the king — rook is never picked/placed
- Consequence: castling silently executes as a king-only move, rook stays put
- File: chess_arm_node.py execute_move()

**🔴 En passant not handled**
- Captured pawn is on a different square than the destination — arm only removes pieces from `to_name`
- Consequence: captured pawn is never removed from the board
- File: chess_arm_node.py execute_move()

---

## chess_vision_node.py

**🔴 `_sample_perspective()` ignores `board_flip` parameter**
- Uses `7 - ri` and `7 - fi` unconditionally, not checking `self.board_flip`
- `_compute_piece_centroids()` does check `board_flip` — inconsistent
- Consequence: when `board_flip=True`, occupancy detection samples wrong tile positions
- File: chess_vision_node.py ~lines 1177–1179

**🔴 Tiles at frame edge return 0.0 brightness**
- When a tile patch is near the image boundary, the clipped patch is used correctly but if fully outside, returns 0.0
- Consequence: edge tiles may report false "empty" and miss pieces
- File: chess_vision_node.py ~lines 1149–1152

---

## chess_engine_node.py

**🔴 Node continues with uninitialized engine on startup failure**
- If Stockfish path check fails, error is logged but no exception is raised
- Node starts successfully, then crashes with `AttributeError` on first move calculation
- Consequence: cryptic crash mid-game instead of clear startup failure
- File: chess_engine_node.py ~lines 44–57

---

## chess_coord_calibrator.py

**🟢 `_calibrate_cb` blocked the ROS executor**
- Called `_run_calibration()` directly on executor thread; `_snapshot()` uses `time.sleep()` polling
- Stalled entire executor — no callbacks fired during calibration
- Fixed: spawns thread + uses `threading.Event` to wait for result

---

## board_state_node.py

**🔴 No duplicate move protection**
- Same move arriving twice in quick succession (e.g. vision retry) gets applied twice
- Consequence: board state advances two half-moves for one physical move
- File: board_state_node.py ~lines 77–80

---

## chess_gui.py

**🔴 GUI board pushed before camera confirmation**
- `self.board.push(move)` updates local GUI board immediately on user input
- If vision later rejects the move, GUI is already advanced; FEN subscriber will overwrite it but there is a race window
- Consequence: GUI can display illegal board states; user move during race window uses wrong reference
- File: chess_gui.py ~line 245

---

## vision_calib_gui.py

**🟢 `self._clients` dict shadowed rclpy internal `self._clients` list**
- Adding the calibrator's `SetParameters` client to `self._clients` (a dict) hit `AttributeError: 'dict' object has no attribute 'append'` when rclpy tried to register the new client
- Fixed: renamed to `self._param_clients`

**🟢 Duplicate `Trigger` import inside `_trigger_calibration()`**
- `from std_srvs.srv import Trigger as _Trigger` was inside the method body, re-executing on every button press
- The same import already existed in `__init__`
- Fixed: removed duplicate
