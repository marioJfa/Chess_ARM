# 3DOF+ Robot Arm — Design Plan & Progress
> Planning chat started: March 2026
> Stack: ROS2 Jazzy · Gazebo · Fusion 360 · 3D Printing + Metal Tube

---

## Status Legend
- ✅ Decided / Done
- 🔄 In Progress
- ⏳ Pending decision
- ❌ Blocked

---

## Architecture Overview

```
BASE
├── Motor 1 ── direct shaft ──────────► DOF1: Base yaw (Z axis)
├── Motor 2 ── bidirectional spool ───► DOF3: Elbow pitch (external tendon along Link1 OD)
├── Motor 3 ── bidirectional spool ───► DOF4: Wrist pitch (tendon through bore)
├── Motor 5 ── TBD ──────────────────► Gripper A
└── Motor 6 ── TBD ──────────────────► Gripper B

SHOULDER JOINT
└── Motor 4 ── direct drive ─────────► DOF2: Shoulder pitch (X axis)

LINK1 TUBE (shoulder → elbow)
├── Bore:     4x PTFE-lined channels → wrist fwd/bck + gripper A/B
└── Outside:  2x external tendon lines → elbow fwd/bck + guide clips every 50mm

LINK2 TUBE (elbow → wrist)
└── Bore:     4x PTFE lines continue → wrist + gripper only

LINK3 (wrist → gripper)
└── Bore lines exit here into existing gripper design
```

---

## Decisions Log

### ✅ Confirmed

| Topic | Decision |
|---|---|
| Tube spec | 22mm OD, 2mm wall, 18mm ID — metal, cut to length |
| Fasteners | M3 throughout — set screws + plastic heat-set inserts |
| Tube grip method | Radial M3 set screws (3x at 120°) with dimples on tube |
| Connector interface | 4x M3 bolt circle (30mm PCD) + 12mm centering boss |
| Joint axes | All arm joints on parallel X axis (pitch only) — base on Z |
| Tendon return | Antagonistic pair — one bidirectional spool per motor |
| Elbow routing | External tendon lines along Link1 OD with printed guide clips |
| Wrist routing | Through bore — PTFE lined (4x 4mm OD liners in 2×2 pattern) |
| Gripper routing | Through bore alongside wrist lines (4 total bore lines) |
| Shoulder motor | Physically mounted at shoulder joint |
| Total motors | 6 confirmed (3 base + 1 shoulder + 2 gripper) |
| Bore layout | 2×2 square, 6mm c-c spacing, ~3.2mm center gap remaining |

### ⏳ Pending

| Topic | Impact |
|---|---|
| Gripper motor location (base or wrist) | Affects base_od and motor_bay_count |
| Motor model selection | Replaces all NEMA17 placeholders — affects spool, housing, base bay sizing |
| Fishing line spec (lb rating) | Confirms tendon_dia (currently 0.5mm / ~30lb braid assumed) |
| Link lengths final | Currently 200 / 180 / 80mm — confirm after reach envelope test |
| Base bearing type | lazy susan vs thrust bearing — affects base_yaw_bearing dims |

---

## Robot Specs (Current)

| Parameter | Value |
|---|---|
| DOF (arm) | 3 confirmed + gripper (existing design) |
| Total reach | ~460mm (link1 + link2 + link3) |
| Tube OD | 22mm |
| Tube ID | 18mm |
| Base OD | 160mm |
| Base height | 120mm |
| Total motors | 6 |
| Fastener standard | M3 |
| Yaw range | ±170° (340° total) |
| Shoulder range | -10° to +150° |
| Elbow range | 0° to +150° |
| Wrist range | ±90° |

---

## Design Phases

### Phase 1 — Parameters & Architecture ✅
- [x] Tube dimensions defined
- [x] Fastener standard chosen (M3)
- [x] Tendon routing architecture decided (hybrid: direct / external / bore)
- [x] Motor count and placement confirmed (6 motors)
- [x] Joint axes locked (all arm joints parallel X axis)
- [x] Bore cross-section verified (4x PTFE liners fit in 18mm ID)
- [x] Fusion 360 parameter CSV exported (`arm_params_fusion360.csv`)
- [ ] Motor model selected — updates placeholders
- [ ] Gripper motor location decided — updates base sizing

### Phase 2 — Tube Socket Connector 🔄
> Start here in Fusion 360

- [ ] Print bore test slug first (30mm long, 18mm bore, 4x PTFE liner holes)
- [ ] Tune `connector_clearance` (0.3mm nominal) from test print
- [ ] Model socket outer body — 22mm bore + 3.5mm wall
- [ ] Add 3x M3 set screw holes with plastic insert pockets at 120°
- [ ] Add 4x M3 bolt circle on interface face (30mm PCD)
- [ ] Add 12mm centering boss on interface face
- [ ] Add 3x longitudinal stiffening ribs between set screw positions
- [ ] Add 4x PTFE liner channels through socket bore
- [ ] Save as master component — all sockets derive from this

### Phase 3 — Joint Connectors ⏳
> After motor is chosen — motor cavity dimensions needed

- [ ] Shoulder connector — motor mount pocket + bore passthrough + external line anchor point
- [ ] Elbow connector — external line redirect pulleys (2x) + bore passthrough
- [ ] Wrist connector — bore lines exit and route into gripper interface
- [ ] All connectors share Phase 2 socket on tube end

### Phase 4 — External Line Guide Clips ⏳
> After Link1 length is confirmed

- [ ] Printed clip grips Link1 OD (28mm clip OD)
- [ ] Two eyelet holes for elbow tendon lines (2mm ID, 5mm offset from tube)
- [ ] Set screw to lock position along tube
- [ ] 4 clips total along Link1 (one every 50mm)

### Phase 5 — Bidirectional Spool ⏳
> After motor model chosen

- [ ] Two helical grooves wound opposite directions on one spool
- [ ] 3mm axial separation between groove sections
- [ ] Flanged ends (30mm OD flange) to retain line
- [ ] Motor shaft interface (currently placeholder 5mm bore)

### Phase 6 — Base Shell ⏳
> After gripper motor location decided (affects bay count)

- [ ] Outer shell — 160mm OD, 120mm height, 4mm wall
- [ ] 3x motor bays radially arranged (+ 2 more if gripper motors here)
- [ ] Spool routing zone — 30mm vertical space above motors
- [ ] 6x cable exit ports on top plate (2 external + 4 bore)
- [ ] Base yaw bearing pocket (50mm OD / 35mm ID)
- [ ] Bottom plate — separate bolted component for internal access
- [ ] Top plate — 6mm thick, arm mounts under load here
- [ ] Internal DIN rail (80mm) for controller board

### Phase 7 — Electrical Panel & Module Interface ⏳

- [ ] Dedicated panel face on base OD (60mm wide)
- [ ] 3x panel-mount connector cutouts with label bosses
- [ ] 6x M3 bolt ring (140mm PCD) for swappable module face plates
- [ ] First module face plate template (blank)

### Phase 8 — Full Assembly & Joints ⏳

- [ ] Import gripper assembly as external component
- [ ] Ground base component
- [ ] Define revolute joints (DOF1–4) with correct axes
- [ ] Set joint limits from confirmed values
- [ ] Run interference detection
- [ ] Drive joints to test full range of motion

### Phase 9 — ROS2 / Gazebo Sync ⏳

- [ ] Export full assembly as STEP
- [ ] Generate URDF from Fusion (or manual from params)
- [ ] Verify link lengths match `arm_params_fusion360.csv` ROS2 rows
- [ ] Confirm joint axes match Gazebo simulation
- [ ] Model tendons as mimic joints in Gazebo until plugin ready
- [ ] Test against existing ROS2 Jazzy code

---

## Files

| File | Status | Notes |
|---|---|---|
| `arm_params_fusion360.csv` | ✅ Ready to import | Import via Modify → Change Parameters → gear icon |
| `robot_arm_plan.md` | ✅ This file | Update throughout design |
| Fusion 360 design | ⏳ Not started | Begin with bore test slug |
| URDF | ⏳ Not started | After Phase 8 |

---

## Print-First Checklist
> Print these before modeling connectors — they validate your params

1. **Bore test slug** — 30mm long cylinder, 18mm bore, 4x PTFE liner holes in 2×2 at 6mm spacing. Confirms `connector_clearance` and bore layout.
2. **Socket test ring** — just the socket section, 25mm deep. Slide onto actual tube, tighten set screws. Confirms fit before modeling the full connector.
3. **Interface face test disc** — flat disc with bolt circle + centering boss only. Mate two together, confirm alignment before committing to full connector model.

---

## Notes & Decisions to Revisit

- `base_od` may shrink back to 130mm if gripper motors stay at wrist — hold until decided
- Shoulder motor adds mass to Link1's effective inertia — factor into motor torque selection
- External elbow tendons will change effective moment arm slightly if they bow under tension — guide clip spacing (50mm) should prevent this but verify in simulation
- All arm joints on same X axis means the arm can only move in one plane without base yaw — this is intentional and simplifies IK
- Gripper design already has joints modeled — import as external component in Phase 8, do not re-model
