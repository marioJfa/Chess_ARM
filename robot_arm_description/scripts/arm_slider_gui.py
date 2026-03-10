#!/usr/bin/env python3
"""
arm_slider_gui.py
Interactive slider GUI for robot_arm_description.
Sends JointTrajectory commands to arm_controller and gripper_controller.

Usage:
  python3 arm_slider_gui.py
  # or after colcon build:
  ros2 run robot_arm_description arm_slider_gui
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
from tkinter import ttk
import threading
import math


# ── Joint definitions ─────────────────────────────────────────────────────────
ARM_JOINTS = [
    ('base_yaw',      -180,  180,  0,   'deg'),
    ('shoulder_roll',  -45,   45,  0,   'deg'),
    ('shoulder_pitch', -90,  135,  0,   'deg'),
    ('elbow_pitch',   -120,  120,  0,   'deg'),
]

GRIPPER_JOINTS = [
    ('finger_1_joint', 0, 70, 0, 'deg'),
    ('finger_2_joint', 0, 70, 0, 'deg'),
    ('finger_3_joint', 0, 70, 0, 'deg'),
]

PRESETS = {
    'Home':        {'base_yaw': 0,   'shoulder_roll': 0,  'shoulder_pitch': 0,   'elbow_pitch': 0,   'finger_1_joint': 0,  'finger_2_joint': 0,  'finger_3_joint': 0},
    'Reach':       {'base_yaw': 0,   'shoulder_roll': 0,  'shoulder_pitch': 60,  'elbow_pitch': -30, 'finger_1_joint': 0,  'finger_2_joint': 0,  'finger_3_joint': 0},
    'Pick':        {'base_yaw': 30,  'shoulder_roll': 10, 'shoulder_pitch': 70,  'elbow_pitch': -50, 'finger_1_joint': 60, 'finger_2_joint': 60, 'finger_3_joint': 60},
    'Wave':        {'base_yaw': 45,  'shoulder_roll': 0,  'shoulder_pitch': 90,  'elbow_pitch': -90, 'finger_1_joint': 0,  'finger_2_joint': 0,  'finger_3_joint': 0},
    'Grip Close':  {'finger_1_joint': 70, 'finger_2_joint': 70, 'finger_3_joint': 70},
    'Grip Open':   {'finger_1_joint': 0,  'finger_2_joint': 0,  'finger_3_joint': 0},
}


class ArmSliderGUI(Node):

    def __init__(self):
        super().__init__('arm_slider_gui')

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        # Current values in degrees
        self.values = {}
        for name, lo, hi, default, _ in ARM_JOINTS + GRIPPER_JOINTS:
            self.values[name] = default

        self.duration_sec = 1.5
        self._build_gui()

    # ── GUI construction ───────────────────────────────────────────────────────
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title('Robot Arm Controller')
        self.root.configure(bg='#0d1117')
        self.root.resizable(False, False)

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Title.TLabel',
                        background='#0d1117', foreground='#58a6ff',
                        font=('Courier', 11, 'bold'))
        style.configure('Section.TLabel',
                        background='#161b22', foreground='#8b949e',
                        font=('Courier', 9))
        style.configure('Joint.TLabel',
                        background='#161b22', foreground='#e6edf3',
                        font=('Courier', 10))
        style.configure('Value.TLabel',
                        background='#161b22', foreground='#58a6ff',
                        font=('Courier', 10, 'bold'))
        style.configure('Finger.TLabel',
                        background='#161b22', foreground='#e07c3a',
                        font=('Courier', 10))
        style.configure('FingerVal.TLabel',
                        background='#161b22', foreground='#e07c3a',
                        font=('Courier', 10, 'bold'))
        style.configure('TScale',
                        background='#161b22', troughcolor='#30363d',
                        sliderlength=18, sliderrelief='flat')

        pad = {'padx': 14, 'pady': 4}

        # ── Header ──
        header = tk.Frame(self.root, bg='#0d1117', pady=10)
        header.pack(fill='x')
        ttk.Label(header, text='ARM CONTROL PANEL', style='Title.TLabel').pack()
        tk.Label(header, text='robot_arm_description  ·  Gazebo Harmonic',
                 bg='#0d1117', fg='#8b949e', font=('Courier', 8)).pack()

        # ── Arm joints ──
        arm_frame = tk.LabelFrame(self.root, text=' ARM JOINTS ',
                                  bg='#161b22', fg='#58a6ff',
                                  font=('Courier', 9, 'bold'),
                                  bd=1, relief='groove')
        arm_frame.pack(fill='x', padx=12, pady=6)

        self.sliders = {}
        self.value_labels = {}

        for name, lo, hi, default, unit in ARM_JOINTS:
            row = tk.Frame(arm_frame, bg='#161b22')
            row.pack(fill='x', **pad)
            ttk.Label(row, text=f'{name:<20}', style='Joint.TLabel').pack(side='left')
            var = tk.DoubleVar(value=default)
            self.sliders[name] = var
            sl = tk.Scale(row, from_=lo, to=hi, orient='horizontal',
                          variable=var, resolution=1, length=260,
                          bg='#161b22', fg='#e6edf3', troughcolor='#30363d',
                          highlightthickness=0, bd=0, showvalue=False,
                          command=lambda v, n=name: self._on_change(n))
            sl.pack(side='left', padx=6)
            lbl = tk.Label(row, text=f'{default:>5}°', width=6,
                           bg='#161b22', fg='#58a6ff', font=('Courier', 10, 'bold'))
            lbl.pack(side='left')
            self.value_labels[name] = lbl

        # ── Gripper joints ──
        grip_frame = tk.LabelFrame(self.root, text=' GRIPPER (3 FINGERS) ',
                                   bg='#161b22', fg='#e07c3a',
                                   font=('Courier', 9, 'bold'),
                                   bd=1, relief='groove')
        grip_frame.pack(fill='x', padx=12, pady=6)

        for name, lo, hi, default, unit in GRIPPER_JOINTS:
            row = tk.Frame(grip_frame, bg='#161b22')
            row.pack(fill='x', **pad)
            tk.Label(row, text=f'{name:<20}', bg='#161b22', fg='#e07c3a',
                     font=('Courier', 10)).pack(side='left')
            var = tk.DoubleVar(value=default)
            self.sliders[name] = var
            sl = tk.Scale(row, from_=lo, to=hi, orient='horizontal',
                          variable=var, resolution=1, length=260,
                          bg='#161b22', fg='#e07c3a', troughcolor='#2d2010',
                          highlightthickness=0, bd=0, showvalue=False,
                          command=lambda v, n=name: self._on_change(n))
            sl.pack(side='left', padx=6)
            lbl = tk.Label(row, text=f'{default:>5}°', width=6,
                           bg='#161b22', fg='#e07c3a', font=('Courier', 10, 'bold'))
            lbl.pack(side='left')
            self.value_labels[name] = lbl

        # ── Duration ──
        dur_frame = tk.Frame(self.root, bg='#0d1117')
        dur_frame.pack(fill='x', padx=12, pady=4)
        tk.Label(dur_frame, text='Move duration (s):', bg='#0d1117',
                 fg='#8b949e', font=('Courier', 9)).pack(side='left')
        self.dur_var = tk.DoubleVar(value=1.5)
        tk.Scale(dur_frame, from_=0.5, to=5.0, orient='horizontal',
                 variable=self.dur_var, resolution=0.5, length=160,
                 bg='#0d1117', fg='#e6edf3', troughcolor='#30363d',
                 highlightthickness=0, bd=0, showvalue=True,
                 font=('Courier', 8)).pack(side='left', padx=8)

        # ── Send button ──
        tk.Button(self.root, text='▶  SEND COMMAND',
                  command=self._send_all,
                  bg='#0d419d', fg='white', activebackground='#1158c7',
                  font=('Courier', 11, 'bold'), bd=0, pady=8,
                  cursor='hand2').pack(fill='x', padx=12, pady=4)

        # ── Presets ──
        preset_frame = tk.LabelFrame(self.root, text=' PRESETS ',
                                     bg='#161b22', fg='#3fb950',
                                     font=('Courier', 9, 'bold'),
                                     bd=1, relief='groove')
        preset_frame.pack(fill='x', padx=12, pady=6)
        btn_row = tk.Frame(preset_frame, bg='#161b22')
        btn_row.pack(padx=8, pady=6)
        for label, vals in PRESETS.items():
            tk.Button(btn_row, text=label,
                      command=lambda v=vals: self._apply_preset(v),
                      bg='#21262d', fg='#e6edf3', activebackground='#30363d',
                      font=('Courier', 9), bd=0, padx=8, pady=4,
                      cursor='hand2').pack(side='left', padx=3)

        # ── Status bar ──
        self.status_var = tk.StringVar(value='Ready')
        tk.Label(self.root, textvariable=self.status_var,
                 bg='#0d1117', fg='#3fb950', font=('Courier', 9),
                 anchor='w').pack(fill='x', padx=14, pady=(2, 8))

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _on_change(self, name):
        val = int(self.sliders[name].get())
        self.values[name] = val
        self.value_labels[name].config(text=f'{val:>5}°')

    def _apply_preset(self, vals):
        for name, val in vals.items():
            if name in self.sliders:
                self.sliders[name].set(val)
                self.values[name] = val
                self.value_labels[name].config(text=f'{val:>5}°')
        self._send_all()

    def _send_all(self):
        self.duration_sec = self.dur_var.get()
        self._send_arm()
        self._send_gripper()
        self.status_var.set(
            f'Sent — arm + gripper  ({self.duration_sec}s)')

    def _send_arm(self):
        msg = JointTrajectory()
        msg.joint_names = [n for n, *_ in ARM_JOINTS]
        pt = JointTrajectoryPoint()
        pt.positions = [
            math.radians(self.values[n]) for n in msg.joint_names
        ]
        pt.time_from_start = Duration(
            sec=int(self.duration_sec),
            nanosec=int((self.duration_sec % 1) * 1e9)
        )
        msg.points = [pt]
        self.arm_pub.publish(msg)

    def _send_gripper(self):
        msg = JointTrajectory()
        msg.joint_names = [n for n, *_ in GRIPPER_JOINTS]
        pt = JointTrajectoryPoint()
        pt.positions = [
            math.radians(self.values[n]) for n in msg.joint_names
        ]
        pt.time_from_start = Duration(
            sec=int(self.duration_sec),
            nanosec=int((self.duration_sec % 1) * 1e9)
        )
        msg.points = [pt]
        self.gripper_pub.publish(msg)

    def _on_close(self):
        self.root.destroy()
        rclpy.shutdown()

    def run(self):
        # Spin ROS in background thread so GUI stays responsive
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()
        self.root.mainloop()


def main():
    rclpy.init()
    gui = ArmSliderGUI()
    gui.run()


if __name__ == '__main__':
    main()
