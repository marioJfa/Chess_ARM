# Robot Arm Torque & Weight Calculator

Build a web app to calculate torque, weight, and motor requirements for a 3DOF tendon-driven robot arm.

## Stack
- Python + FastAPI backend
- Single-page HTML/CSS/JS frontend (no framework)
- All calculations in `calculator.py`
- Fusion 360 CSV export in `fusion_export.py`

## File Structure
```
arm_calculator/
├── main.py
├── calculator.py
├── fusion_export.py
├── requirements.txt
└── static/
    ├── index.html
    ├── style.css
    └── app.js
```

## Arm Design
- 3DOF + gripper
- Base yaw: ring gear, Z axis
- Shoulder pitch: direct shaft drive at joint
- Elbow pitch: cable from base, runs external along Link1
- Wrist pitch: cable from base, runs through tube bore
- All arm joints on parallel X axes

## Default Parameters (all tunable in UI)

### Geometry (mm)
- link1_length = 200
- link2_length = 180
- link3_length = 80
- tube_od = 22
- tube_wall = 2
- tube_id = tube_od - 2 * tube_wall

### Bearing / Hub (mm)
- shaft_dia = 8
- bearing_od = 22
- bearing_width = 7
- hub_od = 40

### Mass (grams)
- motor_mass = 300
- hub_mass = 80
- gripper_mass = 400
- payload_mass = 500

### Pulley / Gear
- spool_radius = 11 mm
- elbow_pulley_radius = 15 mm
- wrist_pulley_radius = 15 mm
- shoulder_gear_ratio = 1.0
- elbow_gear_ratio = 1.0
- wrist_gear_ratio = 1.0

### Pose (degrees)
- shoulder_angle = 45
- elbow_angle = 90
- wrist_angle = 0

### Line
- line_break_strength = 133 N (30 lb braid)
- line_safety_factor = 3.0
- torque_safety_factor = 1.5

## Calculations

### Mass
- Tube mass = pi/4 * (OD^2 - ID^2) * length * density (aluminum 2700 kg/m3)
- Joint mass = hub_mass (input)
- Total mass breakdown table per component

### Static Torque (respect joint angles)
- Shoulder torque = g * sum(mass_i * horizontal_distance_i from shoulder pivot)
- Elbow torque = g * sum(mass_i * horizontal_distance_i from elbow pivot)
- Wrist torque = g * sum(mass_i * horizontal_distance_i from wrist pivot)
- Apply torque_safety_factor to all results

### Motor Requirements
- Required motor torque = joint_torque / gear_ratio / (pulley_radius / spool_radius)
- Flag red if motor undersized, green if margin > 20%

### Gear Ratio Recommendation
- Given target motor torque (tunable): recommended_ratio = joint_torque / motor_torque
- Given target joint speed 60 deg/s: required RPM = speed * gear_ratio / (pulley_radius / spool_radius)

### Cable Tension
- tension = joint_torque / pulley_radius
- Compare vs line_break_strength / line_safety_factor
- Flag red if over limit

## UI Layout
- Left panel: grouped parameter inputs with sliders
- Right panel: live results updating on every input change
- Center: 2D SVG arm visualizer updating with joint angles (side view)
- Bottom: Export button → downloads arm_params_fusion360.csv

## Fusion 360 CSV Export
Columns: Name,Unit,Expression,Value,Comment,Favorite

Export these rows (values from current calculator inputs):
link1_length, link2_length, link3_length, tube_od, tube_wall, tube_id,
shaft_dia, bearing_od, bearing_width, hub_od,
spool_dia (= spool_radius*2), pulley_radius (= elbow_pulley_radius),
connector_clearance=0.3, connector_socket_depth=25, bolt_circle_dia=30,
base_limit_deg=340, shoulder_limit_neg_deg=-10, shoulder_limit_pos_deg=150,
elbow_limit_neg_deg=0, elbow_limit_pos_deg=150,
wrist_limit_neg_deg=-90, wrist_limit_pos_deg=90

## Start order
1. requirements.txt
2. calculator.py (all physics, pure functions, no web)
3. main.py (FastAPI, single /calculate endpoint + /export endpoint)
4. static/index.html + style.css + app.js
5. Test with default values, verify shoulder torque is largest when arm is horizontal
