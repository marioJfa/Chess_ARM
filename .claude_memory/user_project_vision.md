---
name: Project vision and user intent
description: What this arm is for, how it perceives the world, and long-term goals
type: user
---

**This arm is a real-world robot being developed in simulation first.** Gazebo/ROS is the development environment — the physical arm will be built once the software is proven.

**The arm perceives the world exclusively through its wrist camera.** All state detection (board state, piece positions, obstacles, environment) must go through camera vision. There is no other sensor input.

**Why:** This is a core architectural constraint. Never design around bypassing camera perception — it is the arm's only window to the world.

**How to apply:** When designing features, always ask "how would the camera detect this?" rather than "how do we hardcode this state?" Simulation shortcuts (like trusting GUI input as ground truth) are only valid as development scaffolding — the camera path is always the real path.

---

**Planned use cases beyond chess:**
- Chess-playing arm (current focus)
- Mounted on a robotic car → performs tasks around the house
- General manipulation tasks in a home environment

**Why:** The system needs to be general-purpose and extensible, not chess-specific.

**How to apply:** Avoid hardcoding chess-specific assumptions into low-level arm control or vision infrastructure. Keep chess logic isolated in chess-specific nodes.

---

**Long-term goal: app-controlled arm.** The tuning GUI (ArmTunerGUI, `vision_calib_gui.py`) is the model for how the arm should feel to use — tabbed, live-updating, no restart needed. The eventual mobile/desktop app should feel the same way.

**Why:** User explicitly wants ease-of-use and convenience as a first-class requirement.

**How to apply:** When adding features, always add corresponding GUI controls (per standing rule on vision_calib_gui.py). Design ROS interfaces to be app-friendly (clear topics, clean params, no hidden state).

---

**Camera is the ONLY source of game state updates:**
- The arm plays against a real human player. All move detection comes from the camera exclusively.
- The GUI is a calibration and development tool only — it must NEVER bypass the camera by publishing directly to `/chess/human_move` or updating game state directly.
- The correct GUI-move flow: GUI teleports piece in Gazebo → vision node detects the change → vision publishes `/chess/human_move` → game advances. Vision is always in the loop.
- Never shortcut this by having the GUI publish game moves directly. The camera path is not optional scaffolding — it is the only valid path, in simulation and in real life.
