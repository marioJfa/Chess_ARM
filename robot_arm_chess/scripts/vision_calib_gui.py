#!/usr/bin/env python3
"""
vision_calib_gui.py — Multi-tab arm control & calibration panel.

Tabs:
  Vision      — ArUco homography, grid nudge (±300 px)
  Detection   — piece/change thresholds, sample radius, debug overlay
  Board Setup — board origin, square size, flip  (→ chess_arm_node)
  Movement    — heights, gripper, speed           (→ chess_arm_node)
  Standby     — standby joint angles + Go button  (→ chess_arm_node)
  Commands    — RECAL / REMOVE_PIECES / RESET / RETURN_PIECES / STANDBY

All sliders update live via ROS 2 set_parameters service — no restart needed.
"""

import threading

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import String

import tkinter as tk
from tkinter import ttk

# ── Node names ────────────────────────────────────────────────────────────────
VISION_NODE = '/chess_vision_node'
ARM_NODE    = '/chess_arm_node'

# ── Colour palette ────────────────────────────────────────────────────────────
BG      = '#0d1117'
BG2     = '#161b22'
BG3     = '#21262d'
FG      = '#c9d1d9'
FG_DIM  = '#8b949e'
ACCENT  = '#58a6ff'
ORANGE  = '#f0a500'
GREEN   = '#3fb950'

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
    ('hover_height',       'Hover height  (m)',           0.05, 0.35, 0.005,  0.12,  'float', ARM_NODE),
    ('lift_height',        'Lift height  (m)',            0.05, 0.45, 0.005,  0.20,  'float', ARM_NODE),
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


class ArmTunerGUI(Node):

    def __init__(self):
        super().__init__('arm_tuner_gui')

        self.cmd_pub = self.create_publisher(String, '/chess/cmd', 10)
        self.create_subscription(String, '/chess/arm_status',    self._arm_status_cb,    10)
        self.create_subscription(String, '/chess/vision/status', self._vision_status_cb, 10)

        self._arm_status_str    = 'UNKNOWN'
        self._vision_status_str = 'UNKNOWN'

        self._vision_client = self.create_client(SetParameters, f'{VISION_NODE}/set_parameters')
        self._arm_client    = self.create_client(SetParameters, f'{ARM_NODE}/set_parameters')

        self._clients = {VISION_NODE: self._vision_client, ARM_NODE: self._arm_client}

        self._slider_vars   = {}   # param_name → DoubleVar
        self._value_labels  = {}   # param_name → Label
        self._bool_vars     = {}   # param_name → BooleanVar

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

        self._tab_vision    = self._make_tab(nb, 'Vision')
        self._tab_detection = self._make_tab(nb, 'Detection')
        self._tab_board     = self._make_tab(nb, 'Board Setup')
        self._tab_movement  = self._make_tab(nb, 'Movement')
        self._tab_standby   = self._make_tab(nb, 'Standby')
        self._tab_commands  = self._make_tab(nb, 'Commands')

        nb.add(self._tab_vision,    text='Vision')
        nb.add(self._tab_detection, text='Detection')
        nb.add(self._tab_board,     text='Board Setup')
        nb.add(self._tab_movement,  text='Movement')
        nb.add(self._tab_standby,   text='Standby')
        nb.add(self._tab_commands,  text='Commands')

        self._populate_vision()
        self._populate_detection()
        self._populate_board()
        self._populate_movement()
        self._populate_standby()
        self._populate_commands()

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
        self._add_checkbox(f, 'board_flip', 'Flip board  (swap rank/file)  (board_flip)', ARM_NODE)
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

    def _add_checkbox(self, parent, pname, label, node):
        var = tk.BooleanVar(value=False)
        self._bool_vars[pname] = var
        tk.Checkbutton(
            parent, text=label, variable=var,
            bg=BG, fg=FG, selectcolor=BG3,
            activebackground=BG, activeforeground=ACCENT,
            font=('Courier', 9),
            command=lambda p=pname, n=node: self._set_param(p, var.get(), 'bool', n)
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

    def _arm_status_cb(self, msg):
        self.root.after(0, lambda: self._arm_var.set(f'Arm: {msg.data}'))

    def _vision_status_cb(self, msg):
        self.root.after(0, lambda: self._vision_var.set(f'Vision: {msg.data}'))

    def _on_close(self):
        self.root.destroy()
        rclpy.shutdown()

    # ── Parameter client ──────────────────────────────────────────────────────

    def _set_param(self, name: str, value, kind: str, node: str):
        client = self._clients.get(node)
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
