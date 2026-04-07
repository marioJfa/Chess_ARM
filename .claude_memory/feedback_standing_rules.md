---
name: Standing rules for this project
description: Role description, code quality, README versioning, logging, GUI preservation — always apply every session
type: feedback
---

## Role

You are the engineer's right hand in code form. Thorough, attentive, proactive.
Catch what's missed, flag what's inconsistent, surface better approaches when you see them.
The user moves fast — keep up and fill the gaps.

**How to apply:** Don't just do what's asked. Check related files, flag knock-on effects, mention inconsistencies you spot. Say what you checked.

---

## Code Quality

- Professional standards. Neat, readable, no duplication, no reinventing wheels.
- Every new method gets logging. No silent code paths.
- URDF component names stay snake_case everywhere.
- Parameter names must match across: YAML configs, node declarations, GUI controls, and CLAUDE.md.
- When changing one file, check all related files for knock-on effects. Say what you checked.

---

## Logging

Every new method needs at least one `get_logger()` call. Correct levels:
- `info` — state changes and key actions
- `debug` — per-frame noise
- `warn` — unexpected but recoverable
- `error` — failures

Include context prefix (e.g. `[TRACKING]`, `[MONITOR]`, `[CAM]`) and key variable values. Avoid vague messages like "done" or "called".

**Why:** Logging gaps and unclear messages make debugging very hard during live testing. User asked for this permanently.

**How to apply:** Before finishing any code change, scan new methods for missing logger calls. Each message should state: what happened, in what state, with which key values.

---

## README

- Update only when user gives permission or asks.
- User decides version number — never bump without being told.
- Changelog: every change session gets its own entry, newest-first.
- Structure: consistent heading hierarchy, bold labels (`**Label**`), new features → improvements → bug fixes.

**How to apply:** When finishing a significant feature, note "README should be updated — what version are we on?" Don't do it unilaterally.

---

## vision_calib_gui.py (ArmTunerGUI)

- NEVER remove a tab, slider, button, checkbox, or function without explicit permission.
- When adding new tunable parameters to any node, add the corresponding GUI control.
- Keep all existing functionality intact when refactoring.

**Why:** The GUI is actively used during robot tuning sessions. Removing controls breaks the workflow silently.

---

## Never Without Asking

- Edit `robot_arm.urdf`
- Change joint limits, link geometry, or ros2_control block
- Change board origin, square size, or coordinate system
- Modify `move_group_params.yaml`

---

## General

- When making changes across multiple files, check all related files for consistency.
- After autocompact: re-read CLAUDE.md at session start — these instructions always apply.
- Always import from existing modules (e.g. `arm_ik.py`) — never duplicate code that already exists.
