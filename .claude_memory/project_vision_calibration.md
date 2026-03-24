---
name: Vision calibration parameters and tools
description: Chess vision node calibration — grid alignment, thresholds, diff overlay, calib GUI
type: project
---

## Dedicated calibration GUI
```bash
ros2 run robot_arm_chess vision_calib_gui.py
```
Script: `robot_arm_chess/scripts/vision_calib_gui.py`
Sliders talk directly to `chess_vision_node` via ROS param service — live, no restart.

## Live-tunable ROS parameters

### Grid alignment
```bash
# Shifts where ArUco inner corners map to board corners (in squares). Default 0.322.
# Increase → grid expands outward from markers. Decrease → shrinks inward.
ros2 param set /chess_vision_node aruco_inner_offset 0.322

# Pixel nudge of the whole grid right/left
ros2 param set /chess_vision_node grid_dx 0

# Pixel nudge of the whole grid down/up
ros2 param set /chess_vision_node grid_dy 0
```

### Detection thresholds
```bash
# Piece presence — brightness diff vs empty_ref to count as occupied
ros2 param set /chess_vision_node piece_threshold 22.0

# Change detection — diff between two idle frames to count as "moved"
ros2 param set /chess_vision_node change_threshold 18.0

# Sample patch radius at each square centre (pixels)
ros2 param set /chess_vision_node sample_radius 10

# Show per-square diff numbers on debug image (always on in WAIT_PIECES)
ros2 param set /chess_vision_node debug_diff true
```

## Grid overlay
- When ArUco homography is available: draws **tile boundary lines** from 9×9 projected corners (lands on real tile edges, not centre-to-centre)
- File labels a–h and rank labels 1–8 drawn at board edges
- Falls back to centre-to-centre lines when only Hough grid is available

## Diff overlay colors (WAIT_PIECES always, TRACKING when debug_diff=true)
- **Gray** — diff < ½ threshold (clearly empty)
- **Yellow** — borderline zone (threshold needs adjusting)
- **Green** — diff > threshold (clearly occupied)

## HUD shows `thr=22/18` (piece_threshold / change_threshold)

## RECAL command — re-capture empty board ref without losing board grid
- GUI button: **Recalibrate** (in chess_gui.py and vision_calib_gui.py)
- Or: `ros2 topic pub --once /chess/cmd std_msgs/msg/String "data: 'RECAL'"`
- Works from any state that has a valid grid_centres
