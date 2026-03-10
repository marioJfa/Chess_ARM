#!/usr/bin/env python3
"""
joint_slider_controller.py
Interactive slider GUI to control all arm and gripper joints in real time.
Publishes to /arm_controller/joint_trajectory and /gripper_controller/joint_trajectory.

Usage:
    ros2 run robot_arm_description joint_slider_controller
    # or directly:
    python3 joint_slider_controller.py
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
from tkinter import ttk
import threading
import math


# ── Joint definitions ────────────────────────────────────────────────────────

ARM_JOINTS = [
    ('base_yaw',      -180,  180,  0),
    ('shoulder_roll',  -45,   45,  0),
    ('shoulder_pitch', -90,  135,  0),
    ('elbow_pitch',   -120,  120,  0),
]

GRIPPER_JOINTS = [
    ('finger_1_joint', -70, 0, 0),
    ('finger_2_joint', -70, 0, 0),
    ('finger_3_joint', -70, 0, 0),
]

MOVE_TIME_SEC = 1  # trajectory duration in seconds


# ── ROS 2 Publisher Node ──────────────────────────────────────────────────────

class SliderController(Node):
    def __init__(self):
        super().__init__('joint_slider_controller')
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.get_logger().info('Joint Slider Controller started')

    def send_arm(self, positions_deg):
        msg = JointTrajectory()
        msg.joint_names = [j[0] for j in ARM_JOINTS]
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(p) for p in positions_deg]
        pt.time_from_start = Duration(sec=MOVE_TIME_SEC)
        msg.points = [pt]
        self.arm_pub.publish(msg)

    def send_gripper(self, positions_deg):
        msg = JointTrajectory()
        msg.joint_names = [j[0] for j in GRIPPER_JOINTS]
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(p) for p in positions_deg]
        pt.time_from_start = Duration(sec=MOVE_TIME_SEC)
        msg.points = [pt]
        self.gripper_pub.publish(msg)


# ── Tkinter GUI ───────────────────────────────────────────────────────────────

class SliderGUI:
    def __init__(self, node: SliderController):
        self.node = node

        self.root = tk.Tk()
        self.root.title('Robot Arm — Joint Controller')
        self.root.configure(bg='#0d1117')
        self.root.resizable(False, False)

        # Styling
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TScale', background='#0d1117')

        self._build_ui()

    def _label(self, parent, text, **kwargs):
        return tk.Label(parent, text=text, bg='#0d1117',
                        fg=kwargs.pop('fg', '#8b949e'),
                        font=kwargs.pop('font', ('Courier', 9)),
                        **kwargs)

    def _section(self, parent, title):
        frame = tk.LabelFrame(parent, text=title,
                              bg='#161b22', fg='#58a6ff',
                              font=('Courier', 10, 'bold'),
                              bd=1, relief='solid',
                              padx=10, pady=8)
        return frame

    def _build_ui(self):
        root = self.root
        PAD = dict(padx=12, pady=6)

        # Title
        tk.Label(root, text='ROBOT ARM CONTROLLER',
                 bg='#0d1117', fg='#58a6ff',
                 font=('Courier', 13, 'bold')).pack(pady=(14, 2))
        tk.Label(root, text='ROS 2 Jazzy  ·  Gazebo Harmonic',
                 bg='#0d1117', fg='#444d56',
                 font=('Courier', 8)).pack(pady=(0, 10))

        # ── ARM JOINTS ──
        arm_frame = self._section(root, ' ARM JOINTS ')
        arm_frame.pack(fill='x', **PAD)

        self.arm_vars = []
        self.arm_labels = []
        for name, lo, hi, default in ARM_JOINTS:
            row = tk.Frame(arm_frame, bg='#161b22')
            row.pack(fill='x', pady=3)

            self._label(row, f'{name:<20}', fg='#e6edf3',
                        font=('Courier', 10)).pack(side='left')

            val_lbl = self._label(row, f'{default:>6}°',
                                  fg='#58a6ff',
                                  font=('Courier', 10, 'bold'),
                                  width=7)
            val_lbl.pack(side='right')
            self.arm_labels.append(val_lbl)

            var = tk.DoubleVar(value=default)
            self.arm_vars.append(var)

            sl = tk.Scale(row, from_=lo, to=hi, orient='horizontal',
                          variable=var, resolution=1,
                          bg='#161b22', fg='#58a6ff',
                          troughcolor='#30363d', activebackground='#79c0ff',
                          highlightthickness=0, bd=0,
                          length=280, showvalue=False,
                          command=lambda v, i=len(self.arm_vars)-1: self._arm_changed(i, v))
            sl.pack(side='left', padx=(8, 0))

        # ── GRIPPER ──
        grip_frame = self._section(root, ' GRIPPER ')
        grip_frame.pack(fill='x', **PAD)

        self.grip_vars = []
        self.grip_labels = []
        for name, lo, hi, default in GRIPPER_JOINTS:
            row = tk.Frame(grip_frame, bg='#161b22')
            row.pack(fill='x', pady=3)

            self._label(row, f'{name:<20}', fg='#e6edf3',
                        font=('Courier', 10)).pack(side='left')

            val_lbl = self._label(row, f'{default:>6}°',
                                  fg='#e07c3a',
                                  font=('Courier', 10, 'bold'),
                                  width=7)
            val_lbl.pack(side='right')
            self.grip_labels.append(val_lbl)

            var = tk.DoubleVar(value=default)
            self.grip_vars.append(var)

            sl = tk.Scale(row, from_=lo, to=hi, orient='horizontal',
                          variable=var, resolution=1,
                          bg='#161b22', fg='#e07c3a',
                          troughcolor='#30363d', activebackground='#f0a070',
                          highlightthickness=0, bd=0,
                          length=280, showvalue=False,
                          command=lambda v, i=len(self.grip_vars)-1: self._grip_changed(i, v))
            sl.pack(side='left', padx=(8, 0))

        # ── PRESETS ──
        preset_frame = self._section(root, ' PRESETS ')
        preset_frame.pack(fill='x', **PAD)

        btn_row = tk.Frame(preset_frame, bg='#161b22')
        btn_row.pack()

        presets = [
            ('HOME',        [0,   0,   0,   0],   [0,  0,  0]),
            ('REACH',       [0,   0,  60, -30],   [0,  0,  0]),
            ('PICK READY',  [30,  0,  70, -45],   [0,  0,  0]),
            ('CLOSE GRIP',  None,                  [-60, -60, -60]),
            ('OPEN GRIP',   None,                  [0,   0,   0]),
        ]

        for label, arm_pos, grip_pos in presets:
            btn = tk.Button(btn_row, text=label,
                            bg='#21262d', fg='#e6edf3',
                            activebackground='#30363d', activeforeground='#58a6ff',
                            font=('Courier', 9), bd=0, padx=8, pady=4,
                            cursor='hand2',
                            command=lambda a=arm_pos, g=grip_pos: self._apply_preset(a, g))
            btn.pack(side='left', padx=4, pady=4)

        # ── SEND button ──
        tk.Button(root, text='⟳  SEND ALL',
                  bg='#0d419d', fg='white',
                  activebackground='#1158c7', activeforeground='white',
                  font=('Courier', 11, 'bold'), bd=0, padx=16, pady=8,
                  cursor='hand2',
                  command=self._send_all).pack(pady=(4, 14))

        # Status bar
        self.status_var = tk.StringVar(value='Ready')
        tk.Label(root, textvariable=self.status_var,
                 bg='#0d1117', fg='#3fb950',
                 font=('Courier', 8)).pack(pady=(0, 8))

    def _arm_changed(self, i, val):
        self.arm_labels[i].config(text=f'{int(float(val)):>6}°')
        self._send_arm()

    def _grip_changed(self, i, val):
        self.grip_labels[i].config(text=f'{int(float(val)):>6}°')
        self._send_gripper()

    def _send_arm(self):
        vals = [v.get() for v in self.arm_vars]
        self.node.send_arm(vals)
        self.status_var.set(f'ARM → {[int(v) for v in vals]}°')

    def _send_gripper(self):
        vals = [v.get() for v in self.grip_vars]
        self.node.send_gripper(vals)
        self.status_var.set(f'GRIP → {[int(v) for v in vals]}°')

    def _send_all(self):
        self._send_arm()
        self._send_gripper()

    def _apply_preset(self, arm_pos, grip_pos):
        if arm_pos is not None:
            for var, lbl, val in zip(self.arm_vars, self.arm_labels, arm_pos):
                var.set(val)
                lbl.config(text=f'{val:>6}°')
            self._send_arm()
        if grip_pos is not None:
            for var, lbl, val in zip(self.grip_vars, self.grip_labels, grip_pos):
                var.set(val)
                lbl.config(text=f'{val:>6}°')
            self._send_gripper()

    def run(self):
        self.root.mainloop()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = SliderController()

    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run GUI on main thread (tkinter requirement)
    gui = SliderGUI(node)
    gui.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
