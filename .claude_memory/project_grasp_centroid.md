---
name: grasp centroid architecture
description: Camera-informed grasp position system — current state and TODO for sub-tile back-projection
type: project
---

The pick-and-place system has a camera centroid pipeline wired up but not yet computing real camera positions.

**Current state:**
- `chess_vision_node._compute_piece_centroids()` publishes `/chess/vision/piece_centroids` (JSON `{sq: [world_x, world_y]}`) at every idle stable snapshot — currently returns tile centres (same as `square_to_xyz`)
- `chess_arm_node` subscribes, caches in `self._piece_centroids`, and `pick_piece` uses the cached XY if present (`src=centroid`), falls back to tile centre (`src=tile-centre`)
- `pawn_grasp_height`, `piece_grasp_height`, `grasp_x_offset`, `grasp_y_offset` params + sliders in Movement tab are live-tunable

**TODO (future session):** Replace the 3-line body of `_compute_piece_centroids()` in `chess_vision_node.py` with brightness-weighted back-projection:
1. Warp tile quad → 32×32 patch (same as `_sample_perspective`)
2. Compute abs-deviation-from-empty_ref weighted centroid in patch coords
3. Back-map patch → image pixel via `M_inv = np.linalg.inv(M)`
4. Back-project image pixel → board-space via `H_inv = np.linalg.inv(self.homography)`
5. Convert board-space (bx_norm, by_norm) → world XY: `rank = 7.5 - bx_norm`, `file = 7.5 - by_norm`, `wx = ox + rank*sq`, `wy = oy + file*sq`

**Why:** User wants sub-tile grasp accuracy using camera perception. Tile-centre mode is the approved interim until camera back-projection is validated.
