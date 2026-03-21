#!/usr/bin/env python3
"""
vision_calib_gui.py — Multi-tab arm control & calibration panel.

Tabs:
  Vision      — ArUco homography, grid nudge (±300 px)
  Detection   — piece/change thresholds, sample radius, debug overlay
  Board Setup — board origin, square size, flip  (→ chess_arm_node)
  Movement    — heights, gripper, speed           (→ chess_arm_node)
  Standby     — standby joint angles + Go button  (→ chess_arm_node)
  Move Arm    — send custom XYZ (analytical IK) or direct joint angles
  Commands    — RECAL / REMOVE_PIECES / RESET / RETURN_PIECES / STANDBY
  Console     — live INFO/WARN/ERROR logs from arm + vision nodes

All sliders update live via ROS 2 set_parameters service — no restart needed.
"""

import importlib.util
import json
import os
import threading

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType, Log
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration
from ament_index_python.packages import get_package_share_directory as _gpsd

import tkinter as tk
from tkinter import ttk

PRESET_DIR = os.path.expanduser('~/.ros/arm_tuner_presets')

# ── Arm IK (reuse robot_arm_moveit/scripts/arm_ik.py) ────────────────────────
def _import_arm_ik():
    share   = _gpsd('robot_arm_moveit')
    lib_dir = os.path.join(share, '..', '..', 'lib', 'robot_arm_moveit')
    path    = os.path.normpath(os.path.join(lib_dir, 'arm_ik.py'))
    spec    = importlib.util.spec_from_file_location('arm_ik', path)
    mod     = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

_arm_ik_mod   = _import_arm_ik()
analytical_ik = _arm_ik_mod.analytical_ik

# ── Node names ────────────────────────────────────────────────────────────────
VISION_NODE   = '/chess_vision_node'
ARM_NODE      = '/chess_arm_node'
CALIB_NODE    = '/chess_coord_calibrator'

# ── Colour palette ────────────────────────────────────────────────────────────
BG      = '#0d1117'
BG2     = '#161b22'
BG3     = '#21262d'
FG      = '#c9d1d9'
FG_DIM  = '#8b949e'
ACCENT  = '#58a6ff'
ORANGE  = '#f0a500'
GREEN   = '#3fb950'

# ── Console log level colours ─────────────────────────────────────────────────
LOG_COLORS = {10: '#6e7681', 20: '#c9d1d9', 30: '#f0a500', 40: '#f85149', 50: '#f85149'}
LOG_LABELS = {10: 'DEBUG', 20: 'INFO ', 30: 'WARN ', 40: 'ERROR', 50: 'FATAL'}
# Nodes to show in console (real-world relevant)
CONSOLE_NODES = {'chess_arm_node', 'chess_vision_node'}
LOG_MAX_LINES = 500   # trim buffer to this many lines

# ── Slider definitions ────────────────────────────────────────────────────────
# (param_name, label, min, max, resolution, default, 'float'|'int'|'bool', target_node)
VISION_SLIDERS = [
    ('aruco_inner_offset', 'ArUco inner offset  (squares)', 0.0,  1.5,  0.005, 0.322, 'float', VISION_NODE),
    ('marker_sq_size',     'Marker size  (squares)',        0.5,  2.5,  0.01,  1.333, 'float', VISION_NODE),
    ('grid_dx',            'Grid nudge X  (px)',           -300,  300,  1,     0,     'int',   VISION_NODE),
    ('grid_dy',            'Grid nudge Y  (px)',           -300,  300,  1,     0,     'int',   VISION_NODE),
]

DETECTION_SLIDERS = [
    ('piece_threshold',  'Piece threshold  (diff vs empty ref)',  1.0, 80.0, 0.5, 22.0, 'float', VISION_NODE),
    ('change_threshold', 'Change threshold  (diff vs prev idle)', 1.0, 60.0, 0.5, 18.0, 'float', VISION_NODE),
    ('sample_radius',    'Sample radius  (px)',                   2,   30,   1,   10,   'int',   VISION_NODE),
]

BOARD_SLIDERS = [
    ('origin_x',    'Board origin X  (m)', -0.5, 1.0,  0.001,  0.200, 'float', ARM_NODE),
    ('origin_y',    'Board origin Y  (m)', -0.5, 0.5,  0.001, -0.175, 'float', ARM_NODE),
    ('origin_z',    'Board origin Z  (m)', -0.1, 0.2,  0.001,  0.020, 'float', ARM_NODE),
    ('square_size', 'Square size  (m)',     0.02, 0.10, 0.001,  0.045, 'float', ARM_NODE),
]

MOVEMENT_SLIDERS = [
    ('pawn_grasp_height',  'Pawn grasp height  (m)',      0.0,  0.15, 0.002,  0.04,  'float', ARM_NODE),
    ('piece_grasp_height', 'Piece grasp height  (m)',     0.0,  0.15, 0.002,  0.04,  'float', ARM_NODE),
    ('place_grasp_height', 'Place grasp height  (m)',     0.0,  0.15, 0.002,  0.04,  'float', ARM_NODE),
    ('grasp_x_offset',     'Grasp X offset  (m)',        -0.05, 0.05, 0.001,  0.0,   'float', ARM_NODE),
    ('grasp_y_offset',     'Grasp Y offset  (m)',        -0.05, 0.05, 0.001,  0.0,   'float', ARM_NODE),
    ('grasp_height',       'Grasp height  (m, legacy)',   0.0,  0.15, 0.002,  0.04,  'float', ARM_NODE),
    ('hover_height',       'Hover height  (m)',           0.05, 0.35, 0.005,  0.08,  'float', ARM_NODE),
    ('lift_height',        'Lift height  (m)',            0.05, 0.45, 0.005,  0.15,  'float', ARM_NODE),
    ('gripper_open',       'Gripper open  (rad)',         -1.3, 0.0,  0.01,   0.0,   'float', ARM_NODE),
    ('gripper_closed',     'Gripper closed  (rad)',       -1.3, 0.0,  0.01,  -1.05,  'float', ARM_NODE),
    ('move_duration',      'Move duration  (s)',           0.5, 6.0,  0.1,    2.5,   'float', ARM_NODE),
]

STANDBY_SLIDERS = [
    ('standby_base_yaw',       'Base yaw  (rad)',        -3.14, 3.14, 0.01,  0.015, 'float', ARM_NODE),
    ('standby_shoulder_roll',  'Shoulder roll  (rad)',   -0.78, 0.78, 0.01, -0.3,   'float', ARM_NODE),
    ('standby_shoulder_pitch', 'Shoulder pitch  (rad)',  -1.57, 2.36, 0.01,  1.05,  'float', ARM_NODE),
    ('standby_elbow_pitch',    'Elbow pitch  (rad)',     -2.09, 2.09, 0.01,  0.28,  'float', ARM_NODE),
]

CALIBRATOR_SLIDERS = [
    # Camera-on-wrist offset
    ('cam_offset_x',        'Camera offset X  (m)',      -0.1,  0.2,  0.002, 0.04, 'float', CALIB_NODE),
    ('cam_offset_y',        'Camera offset Y  (m)',      -0.1,  0.1,  0.002, 0.0,  'float', CALIB_NODE),
    ('cam_offset_z',        'Camera offset Z  (m)',      -0.1,  0.2,  0.002, 0.02, 'float', CALIB_NODE),
    # Board geometry
    ('board_z',             'Board Z  (m)',              -0.05, 0.2,  0.002, 0.02, 'float', CALIB_NODE),
    ('board_edge_offset_x', 'Board edge offset X  (m)',  -0.1,  0.3,  0.002, 0.0,  'float', CALIB_NODE),
    ('aruco_inner_offset',  'ArUco inner offset  (sq)',   0.0,  1.5,  0.005, 0.322,'float', CALIB_NODE),
]

ARM_JOINTS     = ['base_yaw', 'shoulder_roll', 'shoulder_pitch', 'elbow_pitch']
GRIPPER_JOINTS = ['finger_1_joint', 'finger_2_joint', 'finger_3_joint']

# Joint jog slider definitions (separate from standby — these drive _publish_arm_traj directly)
JOG_SLIDERS = [
    ('base_yaw',       'Base yaw  (rad)',       -3.14, 3.14, 0.01,  0.015),
    ('shoulder_roll',  'Shoulder roll  (rad)',  -0.785, 0.785, 0.01, -0.3),
    ('shoulder_pitch', 'Shoulder pitch  (rad)', -1.57,  2.36, 0.01,  1.05),
    ('elbow_pitch',    'Elbow pitch  (rad)',    -2.09,  2.09, 0.01,  0.28),
]


class ArmTunerGUI(Node):

    def __init__(self):
        super().__init__('arm_tuner_gui')

        self.cmd_pub = self.create_publisher(String, '/chess/cmd', 10)
        self._arm_traj_pub     = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._gripper_traj_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.create_subscription(String, '/chess/arm_status',    self._arm_status_cb,    10)
        self.create_subscription(String, '/chess/vision/status', self._vision_status_cb, 10)
        self.create_subscription(Log, '/rosout', self._rosout_cb, 100)

        self._arm_status_str    = 'UNKNOWN'
        self._vision_status_str = 'UNKNOWN'
        self._console_autoscroll = True   # pause when user scrolls up

        self._vision_client = self.create_client(SetParameters, f'{VISION_NODE}/set_parameters')
        self._arm_client    = self.create_client(SetParameters, f'{ARM_NODE}/set_parameters')
        self._calib_client  = self.create_client(SetParameters, f'{CALIB_NODE}/set_parameters')

        self._param_clients = {
            VISION_NODE: self._vision_client,
            ARM_NODE:    self._arm_client,
            CALIB_NODE:  self._calib_client,
        }

        # Calibrate service client (std_srvs/Trigger)
        from std_srvs.srv import Trigger as _Trigger
        self._calibrate_srv_client = self.create_client(
            _Trigger, f'{CALIB_NODE}/calibrate')

        self._slider_vars   = {}   # param_name → DoubleVar
        self._value_labels  = {}   # param_name → Label
        self._bool_vars     = {}   # param_name → BooleanVar
        self._jog_vars      = {}   # joint_name → DoubleVar (Move Arm tab)

        self._build_gui()

    # ── GUI construction ──────────────────────────────────────────────────────

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title('Arm Tuner')
        self.root.configure(bg=BG)
        self.root.resizable(True, True)

        # Title bar
        tk.Label(self.root, text='Arm Tuner',
                 bg=BG, fg=ACCENT, font=('Courier', 14, 'bold')).pack(pady=(10, 2))

        # Status row
        status_frame = tk.Frame(self.root, bg=BG)
        status_frame.pack(fill='x', padx=16)
        self._arm_var    = tk.StringVar(value='Arm: UNKNOWN')
        self._vision_var = tk.StringVar(value='Vision: UNKNOWN')
        tk.Label(status_frame, textvariable=self._arm_var,
                 bg=BG, fg=GREEN, font=('Courier', 9)).pack(side='left')
        tk.Label(status_frame, textvariable=self._vision_var,
                 bg=BG, fg=ORANGE, font=('Courier', 9)).pack(side='right')

        # ── Preset toolbar ───────────────────────────────────────────────────
        preset_bar = tk.Frame(self.root, bg=BG2, pady=4)
        preset_bar.pack(fill='x', padx=10, pady=(0, 4))

        tk.Label(preset_bar, text='Preset:', bg=BG2, fg=FG_DIM,
                 font=('Courier', 8)).pack(side='left', padx=(6, 2))

        self._preset_name = tk.StringVar(value='default')
        tk.Entry(preset_bar, textvariable=self._preset_name,
                 bg=BG3, fg=FG, insertbackground=FG,
                 font=('Courier', 9), width=18, relief='flat').pack(side='left', padx=4)

        for label, cmd, color in [
            ('Save',   self._save_preset,   '#1a4a2a'),
            ('Load',   self._load_preset,   '#1a2a4a'),
            ('Delete', self._delete_preset, '#4a1a1a'),
        ]:
            tk.Button(preset_bar, text=label, command=cmd,
                      bg=color, fg='white', font=('Courier', 8),
                      bd=0, padx=10, pady=3,
                      cursor='hand2').pack(side='left', padx=2)

        self._preset_status = tk.StringVar(value='')
        tk.Label(preset_bar, textvariable=self._preset_status,
                 bg=BG2, fg=GREEN, font=('Courier', 8)).pack(side='left', padx=8)

        # ── Notebook ─────────────────────────────────────────────────────────
        style = ttk.Style()
        style.theme_use('default')
        style.configure('TNotebook',       background=BG,  borderwidth=0)
        style.configure('TNotebook.Tab',   background=BG3, foreground=FG_DIM,
                        padding=[10, 4],   font=('Courier', 9, 'bold'))
        style.map('TNotebook.Tab',
                  background=[('selected', BG2)],
                  foreground=[('selected', ACCENT)])
        style.configure('TFrame', background=BG)

        nb = ttk.Notebook(self.root)
        nb.pack(fill='both', expand=True, padx=10, pady=8)

        self._tab_vision     = self._make_tab(nb, 'Vision')
        self._tab_detection  = self._make_tab(nb, 'Detection')
        self._tab_board      = self._make_tab(nb, 'Board Setup')
        self._tab_movement   = self._make_tab(nb, 'Movement')
        self._tab_standby    = self._make_tab(nb, 'Standby')
        self._tab_calibrator = self._make_tab(nb, 'Calibrator')
        self._tab_move       = self._make_tab(nb, 'Move Arm')
        self._tab_commands   = self._make_tab(nb, 'Commands')
        self._tab_console    = self._make_tab(nb, 'Console')

        nb.add(self._tab_vision,     text='Vision')
        nb.add(self._tab_detection,  text='Detection')
        nb.add(self._tab_board,      text='Board Setup')
        nb.add(self._tab_movement,   text='Movement')
        nb.add(self._tab_standby,    text='Standby')
        nb.add(self._tab_calibrator, text='Calibrator')
        nb.add(self._tab_move,       text='Move Arm')
        nb.add(self._tab_commands,   text='Commands')
        nb.add(self._tab_console,    text='Console')

        self._populate_vision()
        self._populate_detection()
        self._populate_board()
        self._populate_movement()
        self._populate_standby()
        self._populate_calibrator()
        self._populate_move_arm()
        self._populate_commands()
        self._populate_console()

        self.root.after(200, self._autoload_default)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _make_tab(self, nb, name):
        frame = tk.Frame(nb, bg=BG, padx=14, pady=10)
        return frame

    # ── Tab content ───────────────────────────────────────────────────────────

    def _populate_vision(self):
        f = self._tab_vision
        self._section(f, 'ArUco Homography')
        for cfg in VISION_SLIDERS[:2]:
            self._add_slider(f, cfg)
        self._section(f, 'Grid pixel nudge')
        for cfg in VISION_SLIDERS[2:]:
            self._add_slider(f, cfg)
        self._reset_btn(f, VISION_SLIDERS)

    def _populate_detection(self):
        f = self._tab_detection
        self._section(f, 'Detection thresholds')
        for cfg in DETECTION_SLIDERS:
            self._add_slider(f, cfg)
        self._section(f, 'Debug overlay')
        self._add_checkbox(f, 'debug_diff', 'Show per-square diff numbers  (debug_diff)', VISION_NODE)
        self._reset_btn(f, DETECTION_SLIDERS)

    def _populate_board(self):
        f = self._tab_board
        self._section(f, 'Board origin & size')
        for cfg in BOARD_SLIDERS:
            self._add_slider(f, cfg)
        self._section(f, 'Orientation')
        self._add_checkbox(f, 'board_flip',        'Flip board — arm  (board_flip → chess_arm_node)',    ARM_NODE)
        self._add_checkbox(f, 'board_flip_vision', 'Flip board — vision  (board_flip → chess_vision_node)', VISION_NODE, ros_name='board_flip')
        self._reset_btn(f, BOARD_SLIDERS)

    def _populate_movement(self):
        f = self._tab_movement
        self._section(f, 'Grasp tuning (per piece type & XY offset)')
        for cfg in MOVEMENT_SLIDERS[:4]:
            self._add_slider(f, cfg)
        self._section(f, 'Heights & gripper')
        for cfg in MOVEMENT_SLIDERS[4:]:
            self._add_slider(f, cfg)
        self._reset_btn(f, MOVEMENT_SLIDERS)

    def _populate_standby(self):
        f = self._tab_standby
        self._section(f, 'Standby joint angles (rad)')
        for cfg in STANDBY_SLIDERS:
            self._add_slider(f, cfg)

        tk.Frame(f, bg=BG, height=10).pack()

        tk.Button(f, text='▶  Go to Standby Now',
                  command=lambda: self._send_cmd('STANDBY'),
                  bg='#1a3a5c', fg='white',
                  font=('Courier', 10, 'bold'), bd=0, padx=16, pady=8,
                  cursor='hand2').pack(anchor='w', pady=4)

        self._reset_btn(f, STANDBY_SLIDERS)

    def _populate_calibrator(self):
        f = self._tab_calibrator
        self._section(f, 'Camera-on-wrist offset (m)')
        for cfg in CALIBRATOR_SLIDERS[:3]:
            self._add_slider(f, cfg)
        self._section(f, 'Board geometry')
        for cfg in CALIBRATOR_SLIDERS[3:]:
            self._add_slider(f, cfg)

        tk.Frame(f, bg=BG, height=10).pack()

        tk.Button(f, text='▶  Calibrate Now',
                  command=self._trigger_calibration,
                  bg='#2a4a1a', fg='white',
                  font=('Courier', 10, 'bold'), bd=0, padx=16, pady=8,
                  cursor='hand2').pack(anchor='w', pady=4)

        self._calib_status_var = tk.StringVar(value='')
        tk.Label(f, textvariable=self._calib_status_var,
                 bg=BG, fg=GREEN, font=('Courier', 9)).pack(anchor='w', pady=(4, 0))

        self._reset_btn(f, CALIBRATOR_SLIDERS)

    def _trigger_calibration(self):
        """Call ~/calibrate service on the calibrator node."""
        if not self._calibrate_srv_client.service_is_ready():
            self._calib_status_var.set('Service not ready — is chess_coord_calibrator running?')
            return
        self._calib_status_var.set('Calibrating...')
        req = _Trigger.Request()
        future = self._calibrate_srv_client.call_async(req)
        future.add_done_callback(self._on_calib_done)

    def _on_calib_done(self, future):
        try:
            res = future.result()
            msg = f'✓ {res.message}' if res.success else f'✗ {res.message}'
        except Exception as e:
            msg = f'Error: {e}'
        self.root.after(0, lambda: self._calib_status_var.set(msg))

    def _populate_move_arm(self):
        f = self._tab_move

        # ── Section A: XYZ Cartesian ─────────────────────────────────────────
        self._section(f, 'XYZ (Cartesian IK)')

        xyz_row = tk.Frame(f, bg=BG)
        xyz_row.pack(fill='x', pady=4)
        for label, attr, default in [('X', '_xyz_x', '0.300'),
                                      ('Y', '_xyz_y', '0.000'),
                                      ('Z', '_xyz_z', '0.150')]:
            tk.Label(xyz_row, text=f'{label}:', bg=BG, fg=FG,
                     font=('Courier', 9)).pack(side='left', padx=(8, 2))
            entry = tk.Entry(xyz_row, bg=BG3, fg=FG, insertbackground=FG,
                             font=('Courier', 9), width=7, relief='flat')
            entry.insert(0, default)
            entry.pack(side='left', padx=(0, 4))
            setattr(self, attr, entry)

        dur_row = tk.Frame(f, bg=BG)
        dur_row.pack(fill='x', pady=2)
        tk.Label(dur_row, text='Duration (s):', bg=BG, fg=FG_DIM,
                 font=('Courier', 8)).pack(side='left', padx=(8, 2))
        self._xyz_dur = tk.Entry(dur_row, bg=BG3, fg=FG, insertbackground=FG,
                                  font=('Courier', 9), width=5, relief='flat')
        self._xyz_dur.insert(0, '2.5')
        self._xyz_dur.pack(side='left')

        self._move_xyz_btn = tk.Button(
            dur_row, text='Move to XYZ',
            command=self._send_xyz,
            bg='#1a3a5c', fg='white',
            font=('Courier', 9, 'bold'), bd=0, padx=12, pady=4,
            cursor='hand2')
        self._move_xyz_btn.pack(side='left', padx=10)

        self._move_status = tk.StringVar(value='—')
        tk.Label(f, textvariable=self._move_status,
                 bg=BG2, fg=FG_DIM, font=('Courier', 8),
                 anchor='w', padx=6, pady=3).pack(fill='x', pady=(2, 6))

        # ── Section B: Joint Angles ───────────────────────────────────────────
        self._section(f, 'Joint Angles (Direct)')

        for jname, label, lo, hi, res, default in JOG_SLIDERS:
            row = tk.Frame(f, bg=BG)
            row.pack(fill='x', pady=1)
            tk.Label(row, text=label, bg=BG, fg=FG,
                     font=('Courier', 8), width=28, anchor='w').pack(side='left')
            var = tk.DoubleVar(value=default)
            self._jog_vars[jname] = var
            val_lbl = tk.Label(row, text=f'{default:.3f}', bg=BG, fg=ORANGE,
                               font=('Courier', 8), width=8)
            val_lbl.pack(side='right')
            slider = tk.Scale(
                row, variable=var,
                from_=lo, to=hi, resolution=res,
                orient='horizontal', length=300,
                bg=BG2, fg=FG, troughcolor=BG3,
                highlightthickness=0, showvalue=False,
                command=lambda v, lbl=val_lbl: lbl.config(text=f'{float(v):.3f}'))
            slider.pack(side='left', padx=6)

        jdur_row = tk.Frame(f, bg=BG)
        jdur_row.pack(fill='x', pady=(6, 2))
        tk.Label(jdur_row, text='Duration (s):', bg=BG, fg=FG_DIM,
                 font=('Courier', 8)).pack(side='left', padx=(8, 2))
        self._joint_dur = tk.Entry(jdur_row, bg=BG3, fg=FG, insertbackground=FG,
                                    font=('Courier', 9), width=5, relief='flat')
        self._joint_dur.insert(0, '2.5')
        self._joint_dur.pack(side='left')

        self._move_joint_btn = tk.Button(
            jdur_row, text='Execute Joints',
            command=self._send_joints,
            bg='#2a1a4a', fg='white',
            font=('Courier', 9, 'bold'), bd=0, padx=12, pady=4,
            cursor='hand2')
        self._move_joint_btn.pack(side='left', padx=10)

    def _populate_commands(self):
        f = self._tab_commands
        self._section(f, 'Vision')
        btn_row1 = tk.Frame(f, bg=BG)
        btn_row1.pack(fill='x', pady=4)
        tk.Button(btn_row1, text='Recalibrate ref  (RECAL)',
                  command=lambda: self._send_cmd('RECAL'),
                  bg='#3a3a1a', fg='white',
                  font=('Courier', 9), bd=0, padx=12, pady=6,
                  cursor='hand2').pack(side='left', padx=4)

        self._section(f, 'Pieces')
        btn_row2 = tk.Frame(f, bg=BG)
        btn_row2.pack(fill='x', pady=4)
        piece_btns = [
            ('Remove pieces',   'REMOVE_PIECES',  '#1a4a2a'),
            ('Return pieces',   'RETURN_PIECES',  '#1a2a4a'),
            ('Reset all',       'RESET',          '#5a1a1a'),
        ]
        for label, cmd, color in piece_btns:
            tk.Button(btn_row2, text=label,
                      command=lambda c=cmd: self._send_cmd(c),
                      bg=color, fg='white',
                      font=('Courier', 9), bd=0, padx=12, pady=6,
                      cursor='hand2').pack(side='left', padx=4)

        self._section(f, 'Arm')
        btn_row3 = tk.Frame(f, bg=BG)
        btn_row3.pack(fill='x', pady=4)
        tk.Button(btn_row3, text='Go to Standby',
                  command=lambda: self._send_cmd('STANDBY'),
                  bg='#1a3a5c', fg='white',
                  font=('Courier', 9), bd=0, padx=12, pady=6,
                  cursor='hand2').pack(side='left', padx=4)

        # Log area
        self._section(f, 'Log')
        self._log_var = tk.StringVar(value='—')
        tk.Label(f, textvariable=self._log_var,
                 bg=BG2, fg=FG_DIM, font=('Courier', 8),
                 anchor='w', wraplength=440, justify='left',
                 padx=6, pady=4).pack(fill='x', pady=2)

    def _populate_console(self):
        f = self._tab_console

        # Toolbar: level filter checkboxes + Clear button
        toolbar = tk.Frame(f, bg=BG2, pady=3)
        toolbar.pack(fill='x', pady=(0, 4))

        tk.Label(toolbar, text='Show:', bg=BG2, fg=FG_DIM,
                 font=('Courier', 8)).pack(side='left', padx=(6, 4))

        self._console_levels = {}
        for level, label, color in [
            (10, 'DEBUG', LOG_COLORS[10]),
            (20, 'INFO',  LOG_COLORS[20]),
            (30, 'WARN',  LOG_COLORS[30]),
            (40, 'ERROR', LOG_COLORS[40]),
        ]:
            var = tk.BooleanVar(value=(level >= 20))   # DEBUG off by default
            self._console_levels[level] = var
            tk.Checkbutton(
                toolbar, text=label, variable=var,
                bg=BG2, fg=color, selectcolor=BG3,
                activebackground=BG2, activeforeground=color,
                font=('Courier', 8)
            ).pack(side='left', padx=2)

        tk.Button(toolbar, text='Clear',
                  command=self._console_clear,
                  bg=BG3, fg=FG_DIM, font=('Courier', 8),
                  bd=0, padx=8, pady=2,
                  cursor='hand2').pack(side='right', padx=6)

        # Scrollable text widget
        txt_frame = tk.Frame(f, bg=BG)
        txt_frame.pack(fill='both', expand=True)

        scrollbar = tk.Scrollbar(txt_frame, bg=BG3, troughcolor=BG)
        scrollbar.pack(side='right', fill='y')

        self._console_text = tk.Text(
            txt_frame,
            bg=BG, fg=FG, font=('Courier', 8),
            wrap='word', state='disabled',
            yscrollcommand=scrollbar.set,
            relief='flat', bd=0,
            selectbackground=BG3)
        self._console_text.pack(side='left', fill='both', expand=True)
        scrollbar.config(command=self._console_text.yview)

        # Configure colour tags for each level
        for level, color in LOG_COLORS.items():
            self._console_text.tag_config(f'lvl{level}', foreground=color)
        self._console_text.tag_config('node', foreground=ACCENT)
        self._console_text.tag_config('dim',  foreground=FG_DIM)

        # Pause auto-scroll when user scrolls up; resume at bottom
        self._console_text.bind('<MouseWheel>', self._console_on_scroll)
        self._console_text.bind('<Button-4>',   self._console_on_scroll)
        self._console_text.bind('<Button-5>',   self._console_on_scroll)
        scrollbar.bind('<B1-Motion>', self._console_on_scroll)

    # ── Widget helpers ────────────────────────────────────────────────────────

    def _section(self, parent, title):
        tk.Label(parent, text=f'── {title} ──',
                 bg=BG, fg=FG_DIM,
                 font=('Courier', 8, 'bold')).pack(anchor='w', pady=(8, 2))

    def _add_slider(self, parent, cfg):
        pname, label, lo, hi, res, default, kind, node = cfg

        row = tk.Frame(parent, bg=BG)
        row.pack(fill='x', pady=1)

        tk.Label(row, text=label, bg=BG, fg=FG,
                 font=('Courier', 8), width=40, anchor='w').pack(side='left')

        var = tk.DoubleVar(value=default)
        self._slider_vars[pname] = var

        txt = f'{int(default)}' if kind == 'int' else f'{default:.3f}'
        val_lbl = tk.Label(row, text=txt, bg=BG, fg=ORANGE,
                           font=('Courier', 8), width=8)
        val_lbl.pack(side='right')
        self._value_labels[pname] = val_lbl

        slider = tk.Scale(
            row, variable=var,
            from_=lo, to=hi, resolution=res,
            orient='horizontal', length=300,
            bg=BG2, fg=FG, troughcolor=BG3,
            highlightthickness=0, showvalue=False,
            command=lambda v, p=pname, k=kind, n=node: self._on_slider(p, v, k, n))
        slider.pack(side='left', padx=6)

    def _add_checkbox(self, parent, pname, label, node, ros_name=None):
        """Add a boolean checkbox.  ros_name overrides the ROS param name sent on the wire
        (useful when the GUI key differs from the actual parameter name, e.g. board_flip_vision)."""
        ros_param = ros_name or pname
        var = tk.BooleanVar(value=False)
        self._bool_vars[pname] = var
        tk.Checkbutton(
            parent, text=label, variable=var,
            bg=BG, fg=FG, selectcolor=BG3,
            activebackground=BG, activeforeground=ACCENT,
            font=('Courier', 9),
            command=lambda rp=ros_param, n=node: self._set_param(rp, var.get(), 'bool', n)
        ).pack(anchor='w', pady=2)

    def _reset_btn(self, parent, cfg_list):
        tk.Button(parent, text='Reset tab to defaults',
                  command=lambda: self._reset_tab(cfg_list),
                  bg=BG3, fg=FG_DIM,
                  font=('Courier', 8), bd=0, padx=8, pady=3,
                  cursor='hand2').pack(anchor='e', pady=(8, 0))

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_slider(self, pname, value, kind, node):
        v = int(float(value)) if kind == 'int' else float(value)
        lbl = str(v) if kind == 'int' else f'{v:.3f}'
        if pname in self._value_labels:
            self._value_labels[pname].config(text=lbl)
        self._set_param(pname, v, kind, node)

    def _reset_tab(self, cfg_list):
        for pname, _, _, _, _, default, kind, node in cfg_list:
            if pname in self._slider_vars:
                self._slider_vars[pname].set(default)
                self._on_slider(pname, default, kind, node)

    def _send_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'CMD → {cmd}')
        if hasattr(self, '_log_var'):
            self.root.after(0, lambda: self._log_var.set(f'Sent: {cmd}'))

    # ── Preset helpers ────────────────────────────────────────────────────────

    def _preset_flash(self, msg: str, color: str = GREEN):
        self._preset_status.set(msg)
        self.root.after(2500, lambda: self._preset_status.set(''))

    def _collect_state(self) -> dict:
        state = {}
        for pname, var in self._slider_vars.items():
            state[pname] = var.get()
        for pname, var in self._bool_vars.items():
            state[pname] = var.get()
        return state

    def _apply_state(self, state: dict):
        all_sliders = VISION_SLIDERS + DETECTION_SLIDERS + BOARD_SLIDERS + MOVEMENT_SLIDERS + STANDBY_SLIDERS
        kind_map = {cfg[0]: (cfg[6], cfg[7]) for cfg in all_sliders}

        for pname, value in state.items():
            if pname in self._slider_vars:
                kind, node = kind_map.get(pname, ('float', None))
                self._slider_vars[pname].set(value)
                if node:
                    self._on_slider(pname, value, kind, node)
            elif pname in self._bool_vars:
                # Map GUI bool key → (ros_param_name, target_node)
                bool_route = {
                    'board_flip':        ('board_flip', ARM_NODE),
                    'board_flip_vision': ('board_flip', VISION_NODE),
                    'debug_diff':        ('debug_diff', VISION_NODE),
                }
                ros_name, node = bool_route.get(pname, (pname, VISION_NODE))
                self._bool_vars[pname].set(bool(value))
                self._set_param(ros_name, bool(value), 'bool', node)

    def _save_preset(self):
        name = self._preset_name.get().strip()
        if not name:
            self._preset_flash('Name required', ORANGE)
            return
        os.makedirs(PRESET_DIR, exist_ok=True)
        path = os.path.join(PRESET_DIR, f'{name}.json')
        with open(path, 'w') as f:
            json.dump(self._collect_state(), f, indent=2)
        self.get_logger().info(f'[PRESET] Saved → {path}')
        self._preset_flash(f'Saved  {name}')

    def _load_preset(self):
        name = self._preset_name.get().strip()
        if not name:
            self._preset_flash('Name required', ORANGE)
            return
        path = os.path.join(PRESET_DIR, f'{name}.json')
        if not os.path.exists(path):
            self._preset_flash(f'Not found: {name}', ORANGE)
            return
        with open(path) as f:
            state = json.load(f)
        self._apply_state(state)
        self.get_logger().info(f'[PRESET] Loaded ← {path}')
        self._preset_flash(f'Loaded  {name}')

    def _delete_preset(self):
        name = self._preset_name.get().strip()
        if not name:
            self._preset_flash('Name required', ORANGE)
            return
        path = os.path.join(PRESET_DIR, f'{name}.json')
        if not os.path.exists(path):
            self._preset_flash(f'Not found: {name}', ORANGE)
            return
        os.remove(path)
        self.get_logger().info(f'[PRESET] Deleted {path}')
        self._preset_flash(f'Deleted  {name}', ORANGE)

    def _autoload_default(self):
        path = os.path.join(PRESET_DIR, 'default.json')
        if os.path.exists(path):
            with open(path) as f:
                state = json.load(f)
            self._apply_state(state)
            self.get_logger().info('[PRESET] Auto-loaded default preset')
            self._preset_flash('Loaded  default')

    # ── Console helpers ───────────────────────────────────────────────────────

    def _rosout_cb(self, msg: Log):
        # Filter to relevant nodes only
        node_name = msg.name.lstrip('/')
        if node_name not in CONSOLE_NODES:
            return
        level = msg.level
        # Filter by level checkbox — use main thread check
        self.root.after(0, lambda: self._console_append(level, node_name, msg.msg))

    def _console_append(self, level: int, node: str, text: str):
        if not hasattr(self, '_console_text'):
            return
        var = self._console_levels.get(level)
        if var is not None and not var.get():
            return   # level filtered out

        label = LOG_LABELS.get(level, f'L{level:2d}')
        line  = f'[{label}] [{node}] {text}\n'

        widget = self._console_text
        widget.config(state='normal')

        # Trim to max lines
        lines = int(widget.index('end-1c').split('.')[0])
        if lines > LOG_MAX_LINES:
            widget.delete('1.0', f'{lines - LOG_MAX_LINES}.0')

        # Insert with colour tags
        widget.insert('end', f'[{label}] ', f'lvl{level}')
        widget.insert('end', f'[{node}] ', 'node')
        widget.insert('end', text + '\n', f'lvl{level}')

        widget.config(state='disabled')

        if self._console_autoscroll:
            widget.see('end')

    def _console_clear(self):
        if not hasattr(self, '_console_text'):
            return
        self._console_text.config(state='normal')
        self._console_text.delete('1.0', 'end')
        self._console_text.config(state='disabled')
        self.get_logger().info('[CONSOLE] Cleared')

    def _console_on_scroll(self, event=None):
        """Pause auto-scroll when user scrolls up; resume when at bottom."""
        if not hasattr(self, '_console_text'):
            return
        # Schedule check after the scroll event is processed
        self.root.after(50, self._console_check_scroll_pos)

    def _console_check_scroll_pos(self):
        if not hasattr(self, '_console_text'):
            return
        pos = self._console_text.yview()
        self._console_autoscroll = (pos[1] >= 0.999)

    # ── Move Arm helpers ──────────────────────────────────────────────────────

    def _send_xyz(self):
        if self._arm_status_str not in ('IDLE', 'UNKNOWN'):
            self._move_status.set('Arm busy')
            return
        try:
            x   = float(self._xyz_x.get())
            y   = float(self._xyz_y.get())
            z   = float(self._xyz_z.get())
            dur = float(self._xyz_dur.get())
        except ValueError:
            self._move_status.set('Invalid input')
            return
        sol = analytical_ik(x, y, z)
        if sol is None:
            self._move_status.set(f'No IK solution  ({x:.3f},{y:.3f},{z:.3f})')
            self.get_logger().warn(f'[MOVE ARM] No IK solution for ({x:.3f},{y:.3f},{z:.3f})')
            return
        self._move_status.set(f'Sending → ({x:.3f},{y:.3f},{z:.3f})')
        self.get_logger().info(f'[MOVE ARM] XYZ ({x:.3f},{y:.3f},{z:.3f})  dur={dur}  sol={sol}')
        threading.Thread(target=self._publish_arm_traj, args=(sol, dur), daemon=True).start()

    def _send_joints(self):
        if self._arm_status_str not in ('IDLE', 'UNKNOWN'):
            self._move_status.set('Arm busy')
            return
        try:
            dur = float(self._joint_dur.get())
        except ValueError:
            dur = 2.5
        sol = {j: self._jog_vars[j].get() for j in ARM_JOINTS}
        self.get_logger().info(f'[MOVE ARM] Joints {sol}  dur={dur}')
        self._move_status.set('Sending joints…')
        threading.Thread(target=self._publish_arm_traj, args=(sol, dur), daemon=True).start()

    def _publish_arm_traj(self, solution: dict, duration: float):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [solution[j] for j in ARM_JOINTS]
        pt.time_from_start = RosDuration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        msg.points = [pt]
        self._arm_traj_pub.publish(msg)
        self.get_logger().info(f'[MOVE ARM] Trajectory published  dur={duration:.1f}s')
        self.root.after(int(duration * 1000) + 500,
                        lambda: self._move_status.set('Done'))

    def _refresh_move_buttons(self):
        if not hasattr(self, '_move_xyz_btn'):
            return
        state = 'normal' if self._arm_status_str in ('IDLE', 'UNKNOWN') else 'disabled'
        self._move_xyz_btn.config(state=state)
        self._move_joint_btn.config(state=state)

    def _arm_status_cb(self, msg):
        self._arm_status_str = msg.data
        self.root.after(0, lambda: self._arm_var.set(f'Arm: {msg.data}'))
        self.root.after(0, self._refresh_move_buttons)

    def _vision_status_cb(self, msg):
        self.root.after(0, lambda: self._vision_var.set(f'Vision: {msg.data}'))

    def _on_close(self):
        self.root.destroy()
        rclpy.shutdown()

    # ── Parameter client ──────────────────────────────────────────────────────

    def _set_param(self, name: str, value, kind: str, node: str):
        client = self._param_clients.get(node)
        if client is None:
            return
        if not client.service_is_ready():
            self.get_logger().warn(f'Param service not ready — {node} running?')
            return

        pv = ParameterValue()
        if kind == 'bool':
            pv.type        = ParameterType.PARAMETER_BOOL
            pv.bool_value  = bool(value)
        elif kind == 'int':
            pv.type          = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        else:
            pv.type         = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)

        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=pv)]
        client.call_async(req)

    # ── Spin ──────────────────────────────────────────────────────────────────

    def run(self):
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    gui = ArmTunerGUI()
    gui.run()


if __name__ == '__main__':
    main()
