import math

G = 9.81  # m/s²
ALUMINUM_DENSITY = 2700  # kg/m³


def tube_mass_g(od_mm: float, wall_mm: float, length_mm: float) -> float:
    """Mass of aluminum tube in grams."""
    od = od_mm / 1000
    id_ = (od_mm - 2 * wall_mm) / 1000
    length = length_mm / 1000
    volume = math.pi / 4 * (od**2 - id_**2) * length  # m³
    return volume * ALUMINUM_DENSITY * 1000  # grams


def calculate(params: dict) -> dict:
    # --- Geometry ---
    L1 = params["link1_length"]  # mm
    L2 = params["link2_length"]
    L3 = params["link3_length"]
    tube_od = params["tube_od"]
    tube_wall = params["tube_wall"]
    tube_id = tube_od - 2 * tube_wall

    shaft_dia = params["shaft_dia"]
    bearing_od = params["bearing_od"]
    bearing_width = params["bearing_width"]
    hub_od = params["hub_od"]

    motor_mass = params["motor_mass"]       # g
    hub_mass = params["hub_mass"]
    gripper_mass = params["gripper_mass"]
    payload_mass = params["payload_mass"]

    spool_radius = params["spool_radius"]           # mm
    elbow_pulley_radius = params["elbow_pulley_radius"]
    wrist_pulley_radius = params["wrist_pulley_radius"]
    shoulder_gear_ratio = params["shoulder_gear_ratio"]
    elbow_gear_ratio = params["elbow_gear_ratio"]
    wrist_gear_ratio = params["wrist_gear_ratio"]

    shoulder_deg = params["shoulder_angle"]
    elbow_deg = params["elbow_angle"]
    wrist_deg = params["wrist_angle"]

    line_break = params["line_break_strength"]      # N
    line_sf = params["line_safety_factor"]
    torque_sf = params["torque_safety_factor"]

    motor_torque_target = params.get("motor_torque_target", 1.5)  # Nm

    # --- Mass breakdown ---
    m_link1 = tube_mass_g(tube_od, tube_wall, L1)
    m_link2 = tube_mass_g(tube_od, tube_wall, L2)
    m_link3 = tube_mass_g(tube_od, tube_wall, L3)
    m_hub_shoulder = hub_mass
    m_hub_elbow = hub_mass
    m_hub_wrist = hub_mass
    # motor at base (shoulder) counts but doesn't contribute to arm torque beyond base
    m_motor = motor_mass

    mass_table = [
        {"component": "Link 1 (tube)",    "mass_g": round(m_link1, 1)},
        {"component": "Link 2 (tube)",    "mass_g": round(m_link2, 1)},
        {"component": "Link 3 (tube)",    "mass_g": round(m_link3, 1)},
        {"component": "Shoulder hub",     "mass_g": round(m_hub_shoulder, 1)},
        {"component": "Elbow hub",        "mass_g": round(m_hub_elbow, 1)},
        {"component": "Wrist hub",        "mass_g": round(m_hub_wrist, 1)},
        {"component": "Gripper",          "mass_g": round(gripper_mass, 1)},
        {"component": "Payload",          "mass_g": round(payload_mass, 1)},
        {"component": "Motor (base)",     "mass_g": round(m_motor, 1)},
    ]
    total_mass_g = sum(r["mass_g"] for r in mass_table)

    # --- Joint angles in radians ---
    # Shoulder angle: 0 = arm pointing straight up, 90 = horizontal
    # Elbow angle: 0 = straight, 90 = folded 90°
    # Wrist compensates to keep tool level (ignored for torque calc simplicity)
    sa = math.radians(shoulder_deg)   # angle of link1 from vertical
    ea = math.radians(elbow_deg)      # angle of link2 relative to link1
    wa = math.radians(wrist_deg)      # angle of link3 relative to link2

    # Horizontal distances from each pivot (in meters)
    # Link1 center of mass from shoulder pivot
    L1_m = L1 / 1000
    L2_m = L2 / 1000
    L3_m = L3 / 1000

    # Positions along arm (x = horizontal, y = vertical from shoulder)
    # Link1: CoM at L1/2 along link1
    # Shoulder angle sa is from vertical, so horizontal = sin(sa)
    x_link1_com = (L1_m / 2) * math.sin(sa)
    x_link1_end = L1_m * math.sin(sa)

    # Elbow pivot position relative to shoulder
    x_elbow = x_link1_end
    # Link2 angle from vertical = sa + ea
    ang2 = sa + ea
    x_link2_com = x_elbow + (L2_m / 2) * math.sin(ang2)
    x_link2_end = x_elbow + L2_m * math.sin(ang2)

    # Wrist pivot position
    x_wrist = x_link2_end
    ang3 = ang2 + wa
    x_link3_com = x_wrist + (L3_m / 2) * math.sin(ang3)
    x_link3_end = x_wrist + L3_m * math.sin(ang3)

    # Gripper + payload at end of link3
    x_tip = x_link3_end

    def kg(g_val):
        return g_val / 1000

    # --- Shoulder torque ---
    # Contributions: link1 CoM, elbow hub at elbow, link2 CoM, wrist hub at wrist,
    #                link3 CoM, gripper+payload at tip, motor mass NOT contributing (at base)
    t_shoulder = G * (
        kg(m_link1) * x_link1_com +
        kg(m_hub_shoulder) * 0 +          # shoulder hub AT shoulder pivot
        kg(m_hub_elbow) * x_elbow +
        kg(m_link2) * x_link2_com +
        kg(m_hub_wrist) * x_wrist +
        kg(m_link3) * x_link3_com +
        kg(gripper_mass) * x_tip +
        kg(payload_mass) * x_tip
    )
    t_shoulder *= torque_sf

    # --- Elbow torque ---
    # Everything distal to elbow pivot
    t_elbow = G * (
        kg(m_link2) * (x_link2_com - x_elbow) +
        kg(m_hub_wrist) * (x_wrist - x_elbow) +
        kg(m_link3) * (x_link3_com - x_elbow) +
        kg(gripper_mass) * (x_tip - x_elbow) +
        kg(payload_mass) * (x_tip - x_elbow)
    )
    t_elbow *= torque_sf

    # --- Wrist torque ---
    t_wrist = G * (
        kg(m_link3) * (x_link3_com - x_wrist) +
        kg(gripper_mass) * (x_tip - x_wrist) +
        kg(payload_mass) * (x_tip - x_wrist)
    )
    t_wrist *= torque_sf

    # --- Motor requirements ---
    spool_r = spool_radius / 1000       # m
    e_pulley_r = elbow_pulley_radius / 1000
    w_pulley_r = wrist_pulley_radius / 1000

    # Shoulder: direct shaft drive
    m_torque_shoulder = t_shoulder / shoulder_gear_ratio
    # Elbow: cable, mechanical advantage = pulley_r / spool_r
    m_torque_elbow = t_elbow / elbow_gear_ratio / (e_pulley_r / spool_r)
    # Wrist: cable same way
    m_torque_wrist = t_wrist / wrist_gear_ratio / (w_pulley_r / spool_r)

    def motor_status(required, target):
        if required > target:
            return "red"
        elif required > target * 0.8:
            return "yellow"
        return "green"

    # --- Cable tension ---
    cable_limit = line_break / line_sf

    tension_elbow = t_elbow / elbow_gear_ratio / e_pulley_r
    tension_wrist = t_wrist / wrist_gear_ratio / w_pulley_r

    def tension_status(t):
        if t > cable_limit:
            return "red"
        elif t > cable_limit * 0.8:
            return "yellow"
        return "green"

    # --- Gear ratio recommendations ---
    rec_shoulder = t_shoulder / motor_torque_target / shoulder_gear_ratio
    rec_elbow = (t_elbow / motor_torque_target) * (spool_r / e_pulley_r)
    rec_wrist = (t_wrist / motor_torque_target) * (spool_r / w_pulley_r)

    # Required RPM for 60 deg/s joint speed
    joint_speed_rad = math.radians(60)
    rpm_shoulder = joint_speed_rad * shoulder_gear_ratio * 60 / (2 * math.pi)
    rpm_elbow = joint_speed_rad * elbow_gear_ratio * (e_pulley_r / spool_r) * 60 / (2 * math.pi)
    rpm_wrist = joint_speed_rad * wrist_gear_ratio * (w_pulley_r / spool_r) * 60 / (2 * math.pi)

    # Arm pose for SVG (pass through)
    pose = {
        "shoulder_deg": shoulder_deg,
        "elbow_deg": elbow_deg,
        "wrist_deg": wrist_deg,
        "L1": L1, "L2": L2, "L3": L3,
        "x_elbow_m": round(x_elbow, 4),
        "x_wrist_m": round(x_wrist, 4),
        "x_tip_m": round(x_tip, 4),
    }

    return {
        "tube_id": round(tube_id, 2),
        "mass_table": mass_table,
        "total_mass_g": round(total_mass_g, 1),
        "torques": {
            "shoulder": round(t_shoulder, 3),
            "elbow": round(t_elbow, 3),
            "wrist": round(t_wrist, 3),
        },
        "motor_torques": {
            "shoulder": round(m_torque_shoulder, 3),
            "elbow": round(m_torque_elbow, 3),
            "wrist": round(m_torque_wrist, 3),
            "shoulder_status": motor_status(m_torque_shoulder, motor_torque_target),
            "elbow_status": motor_status(m_torque_elbow, motor_torque_target),
            "wrist_status": motor_status(m_torque_wrist, motor_torque_target),
        },
        "cable": {
            "limit_N": round(cable_limit, 2),
            "elbow_tension_N": round(tension_elbow, 2),
            "wrist_tension_N": round(tension_wrist, 2),
            "elbow_status": tension_status(tension_elbow),
            "wrist_status": tension_status(tension_wrist),
        },
        "gear_recs": {
            "shoulder": round(rec_shoulder, 2),
            "elbow": round(rec_elbow, 2),
            "wrist": round(rec_wrist, 2),
            "rpm_shoulder": round(rpm_shoulder, 1),
            "rpm_elbow": round(rpm_elbow, 1),
            "rpm_wrist": round(rpm_wrist, 1),
        },
        "pose": pose,
    }
