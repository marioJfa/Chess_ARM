---
name: Standing rules for this project
description: Persistent working instructions — README updates, logging, GUI preservation
type: feedback
---

**README updates require user permission and user-decided version number.** Never bump version number autonomously — always ask "what version are we on?" before updating README.

**Why:** User owns versioning decisions. README should reflect intentional milestones, not be bumped on every commit.

**How to apply:** When finishing a significant feature or fix, note "README should be updated — what version are we on?" rather than doing it unilaterally.

---

**Keep README and changelog clearly ordered and structured.** Use consistent heading hierarchy, bold labels on every bullet (`**Label**`), group related items under sub-bullets, order changelog entries newest-first. Within each version entry, order: new features → improvements → bug fixes.

**Why:** User explicitly asked for clear structure permanently.

**How to apply:** Every time the README or changelog is touched, enforce this structure. Reorganise any section being edited to comply — don't leave mixed or flat bullet lists.

---

**Always add logging to new code, and keep log messages clearly ordered and structured.** Every new method needs at least one `get_logger()` call. Use correct levels: `info` for state changes/key actions, `debug` for per-frame data, `warn` for unexpected-but-recoverable, `error` for failures. Log messages must be consistent and easy to follow — include the node/state context prefix (e.g. `[TRACKING]`, `[MONITOR]`, `[CAM]`) and key variable values so logs read as a clear ordered narrative.

**Why:** Logging gaps and unordered/unclear log messages make debugging very hard during live testing. User has explicitly asked for clear log order permanently.

**How to apply:** Before finishing any code change, scan new methods for missing logger calls and add them. Review log messages for clarity — each should state what happened, in what state, and with which key values. Avoid vague messages like "done" or "called".

---

**Never remove anything from vision_calib_gui.py (ArmTunerGUI) without explicit permission.** No tab, slider, button, checkbox, or function may be removed. When adding new tunable params to nodes, add corresponding GUI controls.

**Why:** The GUI is a tool the user actively uses during robot tuning sessions. Removing controls breaks their workflow silently.

**How to apply:** When refactoring the GUI, only add — never remove. If something seems redundant, ask first.
