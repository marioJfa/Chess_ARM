#!/usr/bin/env python3
"""
chess_arm_node.py
Converts chess moves (UCI format) into pick-and-place arm trajectories.
Uses analytical IK to send goals to arm_controller.
Teleports Gazebo piece models via gz service to match arm motion.

Subscribes:
  /chess/engine_move   (std_msgs/String) — arm's move in UCI (e.g. e7e5)
  /chess/board_state   (std_msgs/String) — FEN for capture detection
  /chess/human_move    (std_msgs/String) — vision-confirmed human move: update piece_map
  /chess/gui_move      (std_msgs/String) — GUI-submitted move: Gazebo teleport only, no game trigger

Publishes:
  /chess/arm_move      (std_msgs/String) — confirms move executed
  /chess/arm_status    (std_msgs/String) — IDLE / MOVING / DONE / ERROR
"""

import json
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import SetParametersResult
import math
import chess
import threading
import time
import subprocess

# ── Import analytical_ik from robot_arm_moveit ───────────────────────────────
sys.path.insert(0, os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'robot_arm_moveit', 'lib', 'robot_arm_moveit')))
from arm_ik import analytical_ik, forward_kinematics  # noqa: E402


ARM_JOINTS     = ['base_yaw', 'shoulder_roll', 'shoulder_pitch', 'elbow_pitch']
GRIPPER_JOINTS = ['finger_1_joint', 'finger_2_joint', 'finger_3_joint']

# Piece z-center above ground when sitting on the board (board top = 0.02)
PIECE_Z  = 0.038   # back-rank pieces (cylinder length 0.035)
PAWN_Z   = 0.035   # pawns (cylinder length 0.028)


class ChessArmNode(Node):

    def __init__(self):
        super().__init__('chess_arm_node')

        # Board geometry params
        self.declare_parameter('origin_x',      0.20)
        self.declare_parameter('origin_y',     -0.175)
        self.declare_parameter('origin_z',      0.02)
        self.declare_parameter('square_size',   0.045)
        self.declare_parameter('grasp_height',      0.04)   # legacy — maps to piece_grasp_height
        self.declare_parameter('pawn_grasp_height',  0.04)   # grasp depth for pawns
        self.declare_parameter('piece_grasp_height', 0.04)   # grasp depth for back-rank pieces
        self.declare_parameter('place_grasp_height', 0.04)   # grasp depth when placing
        self.declare_parameter('grasp_x_offset',     0.0)    # world-frame XY nudge at grasp step
        self.declare_parameter('grasp_y_offset',     0.0)
        self.declare_parameter('lift_height',        0.20)
        self.declare_parameter('hover_height',       0.12)
        self.declare_parameter('gripper_open',       0.0)
        self.declare_parameter('gripper_closed',     1.05)
        self.declare_parameter('move_duration',      2.5)
        self.declare_parameter('board_flip', False)

        # Standby pose params (live-tunable from GUI)
        self.declare_parameter('standby_base_yaw',       0.015)
        self.declare_parameter('standby_shoulder_roll',  -0.3)
        self.declare_parameter('standby_shoulder_pitch',  1.05)
        self.declare_parameter('standby_elbow_pitch',     0.28)

        self.ox         = self.get_parameter('origin_x').value
        self.oy         = self.get_parameter('origin_y').value
        self.oz         = self.get_parameter('origin_z').value
        self.sq         = self.get_parameter('square_size').value
        self.board_flip = self.get_parameter('board_flip').value
        self.z_grasp         = self.get_parameter('grasp_height').value       # legacy
        self.z_pawn_grasp    = self.get_parameter('pawn_grasp_height').value
        self.z_piece_grasp   = self.get_parameter('piece_grasp_height').value
        self.z_place_grasp   = self.get_parameter('place_grasp_height').value
        self.grasp_x_offset  = self.get_parameter('grasp_x_offset').value
        self.grasp_y_offset  = self.get_parameter('grasp_y_offset').value
        self.z_lift          = self.get_parameter('lift_height').value
        self.z_hover         = self.get_parameter('hover_height').value
        self.g_open          = self.get_parameter('gripper_open').value
        self.g_close         = self.get_parameter('gripper_closed').value
        self.dur             = self.get_parameter('move_duration').value

        self.sb_base_yaw       = self.get_parameter('standby_base_yaw').value
        self.sb_shoulder_roll  = self.get_parameter('standby_shoulder_roll').value
        self.sb_shoulder_pitch = self.get_parameter('standby_shoulder_pitch').value
        self.sb_elbow_pitch    = self.get_parameter('standby_elbow_pitch').value

        self.add_on_set_parameters_callback(self._on_param_change)

        self.board = chess.Board()
        self.busy  = False

        # Piece tracking: square_name -> gz model name  e.g. 'e2' -> 'wp_pe2'
        self._piece_map        = {}
        self._grave_idx        = 0
        self._pending_gui_moves = set()  # UCI moves already handled by gui_move_cb
        self._init_piece_map()

        self._latest_joint_states = None   # updated by /joint_states subscriber
        self._piece_centroids     = {}     # sq → (world_x, world_y) from vision node

        # Publishers
        self.arm_pub     = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.confirm_pub = self.create_publisher(String, '/chess/arm_move', 10)
        self.status_pub  = self.create_publisher(String, '/chess/arm_status', 10)

        # Subscribers
        from sensor_msgs.msg import JointState
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10)
        self.move_sub = self.create_subscription(
            String, '/chess/engine_move', self.engine_move_cb, 10)
        self.board_sub = self.create_subscription(
            String, '/chess/board_state', self.board_state_cb, 10)
        self.human_sub = self.create_subscription(
            String, '/chess/human_move', self.human_move_cb, 10)
        self.gui_sub = self.create_subscription(
            String, '/chess/gui_move', self.gui_move_cb, 10)
        self.cmd_sub = self.create_subscription(
            String, '/chess/cmd', self.cmd_cb, 10)
        self.centroids_sub = self.create_subscription(
            String, '/chess/vision/piece_centroids', self._centroids_cb, 10)

        self.publish_status('IDLE')
        self.get_logger().info('Chess arm node ready')

        # Move to standby once controllers are fully loaded (~6s after node start)
        self._standby_timer = self.create_timer(6.0, self._go_standby_once)

    # ── Live param updates ────────────────────────────────────────────────────
    def _on_param_change(self, params):
        for p in params:
            n, v = p.name, p.value
            if n == 'origin_x':              self.ox              = v
            elif n == 'origin_y':            self.oy              = v
            elif n == 'origin_z':            self.oz              = v
            elif n == 'square_size':         self.sq              = v
            elif n == 'board_flip':          self.board_flip      = v
            elif n == 'grasp_height':        self.z_grasp         = v   # legacy
            elif n == 'pawn_grasp_height':   self.z_pawn_grasp    = v
            elif n == 'piece_grasp_height':  self.z_piece_grasp   = v
            elif n == 'place_grasp_height':  self.z_place_grasp   = v
            elif n == 'grasp_x_offset':      self.grasp_x_offset  = v
            elif n == 'grasp_y_offset':      self.grasp_y_offset  = v
            elif n == 'lift_height':         self.z_lift          = v
            elif n == 'hover_height':        self.z_hover         = v
            elif n == 'gripper_open':        self.g_open          = v
            elif n == 'gripper_closed':      self.g_close         = v
            elif n == 'move_duration':       self.dur             = v
            elif n == 'standby_base_yaw':       self.sb_base_yaw       = v
            elif n == 'standby_shoulder_roll':  self.sb_shoulder_roll  = v
            elif n == 'standby_shoulder_pitch': self.sb_shoulder_pitch = v
            elif n == 'standby_elbow_pitch':    self.sb_elbow_pitch    = v
        return SetParametersResult(successful=True)

    # ── Piece map ──────────────────────────────────────────────────────────────
    def _init_piece_map(self):
        """Rebuild chess square → Gazebo model name mapping from scratch."""
        self._piece_map = {}   # clear stale entries from any previous game
        back = ['r', 'n', 'b', 'q', 'k', 'b', 'n', 'r']
        files = 'abcdefgh'
        for i, p in enumerate(back):
            self._piece_map[f'{files[i]}1'] = f'wp_{p}{files[i]}1'
            self._piece_map[f'{files[i]}8'] = f'bp_{p}{files[i]}8'
        for i in range(8):
            self._piece_map[f'{files[i]}2'] = f'wp_p{files[i]}2'
            self._piece_map[f'{files[i]}7'] = f'bp_p{files[i]}7'

    def cmd_cb(self, msg: String):
        cmd = msg.data.strip()
        if cmd == 'RESET':
            self._reset_all_pieces()
        elif cmd == 'RECAL':
            # Full reset first so game state matches visuals after calibration
            self._reset_all_pieces()
            self._remove_all_pieces()
        elif cmd == 'REMOVE_PIECES':
            self._remove_all_pieces()
        elif cmd == 'RETURN_PIECES':
            self._return_pieces_to_board()
        elif cmd == 'STANDBY':
            threading.Thread(target=self._go_standby_cmd, daemon=True).start()

    def _go_standby_cmd(self):
        if self.busy:
            self.get_logger().warn('Arm busy — STANDBY ignored')
            return
        self.get_logger().info('STANDBY command received')
        self.send_arm(self._standby_pose(), duration=2.0)
        self._wait_for_arm_stop()

    def _reset_all_pieces(self):
        """Teleport all 32 pieces back to their starting squares and reset game state.
        Always uses flip=True so white pieces land at the far end from the arm (ranks 7-8
        in physical space) and black pieces near the arm, independent of board_flip."""
        self._init_piece_map()          # rebuild map to starting configuration
        for sq_name, model in self._piece_map.items():
            sq = chess.parse_square(sq_name)
            x, y, _ = self.square_to_xyz(sq, flip=True)
            self._teleport(model, x, y, self._piece_z(model))
        self.board = chess.Board()
        self._grave_idx = 0
        self._pending_gui_moves.clear()
        self.busy = False
        self.publish_status('IDLE')
        self.get_logger().info('[RESET] All pieces reset to starting positions')

    def _remove_all_pieces(self):
        """Teleport all 32 pieces off-board for vision calibration."""
        files = 'abcdefgh'
        back  = ['r', 'n', 'b', 'q', 'k', 'b', 'n', 'r']
        models = []
        for i, p in enumerate(back):
            models.append((f'wp_{p}{files[i]}1', PIECE_Z))
            models.append((f'bp_{p}{files[i]}8', PIECE_Z))
        for i in range(8):
            models.append((f'wp_p{files[i]}2', PAWN_Z))
            models.append((f'bp_p{files[i]}7', PAWN_Z))
        for idx, (model, _) in enumerate(models):
            # Park pieces in a row behind the board (negative y)
            x = self.ox + (idx % 16) * 0.045
            y = self.oy - 0.15 - (idx // 16) * 0.06
            self._teleport(model, x, y, 0.05)
        self.get_logger().info(f'Removed {len(models)} pieces for vision calibration')

    def _return_pieces_to_board(self):
        """Teleport each tracked piece back to its current square (no game reset)."""
        count = 0
        for sq_name, model in self._piece_map.items():
            sq = chess.parse_square(sq_name)
            x, y, _ = self.square_to_xyz(sq)
            self._teleport(model, x, y, self._piece_z(model))
            count += 1
        self.get_logger().info(f'Returned {count} pieces to current board positions')

    def _teleport(self, model_name, x, y, z):
        """Move a Gazebo model to (x, y, z) via gz service."""
        req = (f'name: "{model_name}", '
               f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
               f'orientation: {{x: 0, y: 0, z: 0, w: 1}}')
        subprocess.Popen(
            ['gz', 'service', '-s', '/world/arm_world/set_pose',
             '--reqtype', 'gz.msgs.Pose',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '500', '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _teleport_to_graveyard(self, model_name):
        """Move a captured piece off the board."""
        idx = self._grave_idx
        self._grave_idx += 1
        col = idx % 8
        row = idx // 8
        # Dump pieces to the right of the board
        x = self.ox + col * self.sq
        y = self.oy + 8 * self.sq + 0.06 + row * 0.045
        self._teleport(model_name, x, y, self._piece_z(model_name))

    def _piece_z(self, model_name):
        """Return correct z for a piece model sitting on the board."""
        return PAWN_Z if '_p' in model_name[3:] else PIECE_Z

    def _grasp_height(self, model_name: str | None) -> float:
        """Return per-piece-type grasp height.  Pawns use pawn_grasp_height,
        all other pieces use piece_grasp_height.  Falls back to legacy
        grasp_height if the per-type param is still at its default (0.04)."""
        if model_name and '_p' in model_name[3:]:
            return self.z_pawn_grasp
        return self.z_piece_grasp

    def _apply_move_to_map(self, move: chess.Move):
        """Update piece_map for any move (human or engine)."""
        from_name = chess.square_name(move.from_square)
        to_name   = chess.square_name(move.to_square)
        # Remove captured piece from map (teleport handled separately)
        self._piece_map.pop(to_name, None)
        # Move piece
        model = self._piece_map.pop(from_name, None)
        if model:
            self._piece_map[to_name] = model

    def _joint_states_cb(self, msg):
        self._latest_joint_states = msg

    def _wait_for_arm_stop(self, vel_threshold=0.01, timeout=15.0):
        """Block until all arm joints are stationary (velocity < threshold).
        Called after the last trajectory of a sequence so IDLE is only published
        once the arm has physically stopped moving."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            js = self._latest_joint_states
            if js is not None:
                vel_map = dict(zip(js.name, js.velocity))
                vels = [abs(vel_map.get(j, 1.0)) for j in ARM_JOINTS]
                if all(v < vel_threshold for v in vels):
                    self.get_logger().info(
                        f'Arm stopped  vels={[f"{v:.4f}" for v in vels]}  — waiting 1s to settle')
                    time.sleep(1.0)
                    return
            time.sleep(0.05)
        self.get_logger().warn('_wait_for_arm_stop: timeout — publishing IDLE anyway')

    # ── Board geometry ─────────────────────────────────────────────────────────
    def square_to_xyz(self, square: chess.Square, z_offset: float = 0.0, flip: bool = None):
        """Convert a chess square to world XYZ.
        flip overrides self.board_flip when explicitly set (used by reset to
        place pieces at their physical home squares regardless of arm orientation)."""
        file = chess.square_file(square)
        rank = chess.square_rank(square)
        do_flip = self.board_flip if flip is None else flip
        if do_flip:
            x = self.ox + (7 - rank) * self.sq + self.sq / 2
            y = self.oy + (7 - file) * self.sq + self.sq / 2
        else:
            x = self.ox + rank * self.sq + self.sq / 2
            y = self.oy + file * self.sq + self.sq / 2
        z = self.oz + z_offset
        return x, y, z

    # ── Trajectory helpers ────────────────────────────────────────────────────
    def send_arm(self, solution: dict, duration: float = None):
        dur = duration or self.dur
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [solution[j] for j in ARM_JOINTS]
        pt.time_from_start = Duration(
            sec=int(dur), nanosec=int((dur % 1) * 1e9))
        msg.points = [pt]
        self.arm_pub.publish(msg)
        time.sleep(dur + 0.3)

    def send_gripper(self, position: float, duration: float = 1.0):
        msg = JointTrajectory()
        msg.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [position] * 3
        pt.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        msg.points = [pt]
        self.gripper_pub.publish(msg)
        time.sleep(duration + 0.2)

    def move_to_xyz(self, x, y, z, duration=None):
        sol = analytical_ik(x, y, z)
        if sol is None:
            self.get_logger().warn(f'[IK] No solution for ({x:.3f},{y:.3f},{z:.3f}) — skipping')
            return
        self.send_arm(sol, duration)

    # ── Pick and place ────────────────────────────────────────────────────────
    def pick_piece(self, square: chess.Square, model_name: str = None):
        gh      = self._grasp_height(model_name)
        sq_name = chess.square_name(square)
        # XY base: camera centroid when available, otherwise tile centre.
        # TODO: fill in camera back-projection here once centroid computation is validated.
        centroid = self._piece_centroids.get(sq_name)
        if centroid:
            cx, cy = centroid
            src = 'centroid'
        else:
            cx, cy, _ = self.square_to_xyz(square)
            src = 'tile-centre'
        self.get_logger().info(
            f'[PICK] {sq_name}  model={model_name}  src={src}  '
            f'xy=({cx:.4f},{cy:.4f})  grasp_h={gh:.3f}  '
            f'xy_off=({self.grasp_x_offset:.3f},{self.grasp_y_offset:.3f})')

        self.send_gripper(self.g_open)
        # Hover — approach from directly above the XY target
        _, _, z_h = self.square_to_xyz(square, self.z_hover)
        self.move_to_xyz(cx, cy, z_h)
        # Grasp — apply tunable XY offset at the lowest point
        _, _, z_g = self.square_to_xyz(square, gh)
        self.move_to_xyz(cx + self.grasp_x_offset, cy + self.grasp_y_offset, z_g, duration=1.5)
        self.send_gripper(self.g_close)
        _, _, z_l = self.square_to_xyz(square, self.z_lift)
        self.move_to_xyz(cx, cy, z_l, duration=1.5)

    def place_piece(self, square: chess.Square, model_name: str = None):
        gh = self._grasp_height(model_name)
        self.get_logger().info(
            f'[PLACE] {chess.square_name(square)}  model={model_name}  '
            f'grasp_h={gh:.3f}  xy_off=({self.grasp_x_offset:.3f},{self.grasp_y_offset:.3f})')
        x, y, z = self.square_to_xyz(square, self.z_hover)
        self.move_to_xyz(x, y, z)
        # Lower to place depth with tunable XY offset
        x, y, z = self.square_to_xyz(square, gh)
        self.move_to_xyz(x + self.grasp_x_offset, y + self.grasp_y_offset, z, duration=1.5)
        # Teleport piece to destination as the arm lowers to place it
        if model_name:
            px, py, _ = self.square_to_xyz(square)
            self._teleport(model_name, px, py, self._piece_z(model_name))
        self.send_gripper(self.g_open)
        x, y, z = self.square_to_xyz(square, self.z_lift)
        self.move_to_xyz(x, y, z, duration=1.5)

    def remove_captured_piece(self, square: chess.Square, model_name: str = None):
        self.get_logger().info(f'[CAPTURE] Removing {model_name} from {chess.square_name(square)}')
        if model_name:
            self._teleport_to_graveyard(model_name)
        self.pick_piece(square, model_name)
        self.move_to_xyz(self.ox - 0.10, self.oy - 0.05, self.z_hover)
        self.send_gripper(self.g_open)
        self.move_to_xyz(self.ox - 0.10, self.oy - 0.05, self.z_lift)

    def return_home(self):
        self.send_arm(self._standby_pose(), duration=2.0)

    def _standby_pose(self):
        return {
            'base_yaw':       self.sb_base_yaw,
            'shoulder_roll':  self.sb_shoulder_roll,
            'shoulder_pitch': self.sb_shoulder_pitch,
            'elbow_pitch':    self.sb_elbow_pitch,
        }

    def _go_standby_once(self):
        self._standby_timer.cancel()
        self.get_logger().info('=== Init: moving arm to standby ===')
        self.send_arm(self._standby_pose(), duration=2.5)
        self._wait_for_arm_stop()
        self.publish_status('IDLE')

    # ── Vision centroid helper ────────────────────────────────────────────────
    def _wait_for_centroid(self, square: chess.Square, timeout: float = 2.0):
        """Poll _piece_centroids until the square is available, then return (world_x, world_y).
        Falls back to tile centre after timeout."""
        sq_name = chess.square_name(square)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if sq_name in self._piece_centroids:
                cx, cy = self._piece_centroids[sq_name]
                self.get_logger().info(
                    f'[CENTROID] {sq_name} from vision: ({cx:.4f},{cy:.4f})')
                return cx, cy
            time.sleep(0.05)
        cx, cy, _ = self.square_to_xyz(square)
        self.get_logger().warn(
            f'[CENTROID] {sq_name} not available after {timeout:.1f}s '
            f'— falling back to tile centre ({cx:.4f},{cy:.4f})')
        return cx, cy

    # ── Main move execution ───────────────────────────────────────────────────
    def execute_move(self, uci: str):
        self.publish_status('MOVING')
        try:
            move      = chess.Move.from_uci(uci)
            from_sq   = move.from_square
            to_sq     = move.to_square
            from_name = chess.square_name(from_sq)
            to_name   = chess.square_name(to_sq)

            # ── Pre-compute all positions BEFORE moving ───────────────────────
            moving_model = self._piece_map.get(from_name)
            pick_gh  = self._grasp_height(moving_model)   # pawn vs piece pick height
            place_gh = self.z_place_grasp                  # separate place depth

            # From-square XY: wait for vision centroid (falls back to tile centre)
            fx, fy = self._wait_for_centroid(from_sq)
            fx += self.grasp_x_offset
            fy += self.grasp_y_offset

            # To-square XY: tile centre (piece not there yet — no centroid to read)
            tx, ty, _ = self.square_to_xyz(to_sq)

            # All Z levels
            _, _, pick_z_hover  = self.square_to_xyz(from_sq, self.z_hover)
            _, _, pick_z_grasp  = self.square_to_xyz(from_sq, pick_gh)
            _, _, pick_z_lift   = self.square_to_xyz(from_sq, self.z_lift)
            _, _, place_z_hover = self.square_to_xyz(to_sq, self.z_hover)
            _, _, place_z_grasp = self.square_to_xyz(to_sq, place_gh)
            _, _, place_z_lift  = self.square_to_xyz(to_sq, self.z_lift)

            self.get_logger().info(
                f'[PLAN] {uci}  '
                f'pick=({fx:.4f},{fy:.4f},{pick_z_grasp:.4f})  '
                f'place=({tx:.4f},{ty:.4f},{place_z_grasp:.4f})  '
                f'hover_z={pick_z_hover:.3f}  lift_z={pick_z_lift:.3f}')

            # ── Capture first ─────────────────────────────────────────────────
            if self.board.is_capture(move):
                self.get_logger().info('[PLAN] Capture — removing piece first')
                cap_model = self._piece_map.get(to_name)
                self.remove_captured_piece(to_sq, cap_model)

            # ── Pick ──────────────────────────────────────────────────────────
            self.send_gripper(self.g_open)
            self.move_to_xyz(fx, fy, pick_z_hover)
            self.move_to_xyz(fx, fy, pick_z_grasp, duration=1.5)
            self.send_gripper(self.g_close)
            self.move_to_xyz(fx, fy, pick_z_lift, duration=1.5)

            # ── Place ─────────────────────────────────────────────────────────
            self.move_to_xyz(tx, ty, place_z_hover)
            self.move_to_xyz(tx, ty, place_z_grasp, duration=1.5)
            if moving_model:
                px, py, _ = self.square_to_xyz(to_sq)
                self._teleport(moving_model, px, py, self._piece_z(moving_model))
            self.send_gripper(self.g_open)
            self.move_to_xyz(tx, ty, place_z_lift, duration=1.5)

            # ── Finish ────────────────────────────────────────────────────────
            self._apply_move_to_map(move)
            self.return_home()
            self._wait_for_arm_stop()

            msg = String()
            msg.data = uci
            self.confirm_pub.publish(msg)
            self.publish_status('DONE')
            self.get_logger().info(f'[PLAN] Move {uci} complete')

        except Exception as e:
            self.get_logger().error(f'Move execution failed: {e}')
            self.publish_status('ERROR')
        finally:
            self.busy = False
            self.publish_status('IDLE')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def board_state_cb(self, msg: String):
        try:
            self.board = chess.Board(msg.data)
        except ValueError:
            pass

    def engine_move_cb(self, msg: String):
        if self.busy:
            self.get_logger().warn('Arm busy, ignoring move request')
            return
        self.busy = True
        uci = msg.data.strip()
        self.get_logger().info(f'Executing engine move: {uci}')
        thread = threading.Thread(
            target=self.execute_move, args=(uci,), daemon=True)
        thread.start()

    def gui_move_cb(self, msg: String):
        """Teleport a GUI-submitted piece in Gazebo — does NOT update game state.

        The GUI sends moves here so the simulation stays visually in sync.
        chess_vision_node will detect the teleport and publish to /chess/human_move,
        which is what actually triggers the game engine.  This separation ensures
        the arm only responds after camera confirmation — matching real-world behaviour.
        """
        uci = msg.data.strip()
        self.get_logger().info(f'GUI move for Gazebo teleport: {uci}')
        try:
            move      = chess.Move.from_uci(uci)
            from_name = chess.square_name(move.from_square)
            to_name   = chess.square_name(move.to_square)

            cap_model    = self._piece_map.get(to_name)
            moving_model = self._piece_map.get(from_name)

            if not moving_model:
                self.get_logger().warn(
                    f'gui_move_cb: no model for {from_name} — piece_map may be stale')
                return

            # Teleport captured piece to graveyard — only if cap_model belongs to
            # the opponent (prevents graveyard-ing a friendly piece on the target square).
            mover_is_white = moving_model.startswith('wp_')
            if (cap_model and cap_model != moving_model and
                    mover_is_white == cap_model.startswith('bp_')):
                self.get_logger().info(f'GUI capture: {cap_model} on {to_name} → graveyard')
                self._teleport_to_graveyard(cap_model)

            # Teleport moving piece to destination
            px, py, _ = self.square_to_xyz(move.to_square)
            self._teleport(moving_model, px, py, self._piece_z(moving_model))
            self.get_logger().info(
                f'GUI teleport: {moving_model}  {from_name} → {to_name}  ({px:.3f}, {py:.3f})')

            # Update piece map now so it stays consistent with Gazebo state.
            # Register the UCI so human_move_cb (triggered by vision) knows to
            # skip the re-teleport and re-map-update for this move.
            self._apply_move_to_map(move)
            self._pending_gui_moves.add(uci)
            self.get_logger().info(
                f'GUI teleport complete — awaiting vision confirmation  '
                f'pending={self._pending_gui_moves}')

        except Exception as e:
            self.get_logger().warn(f'gui_move_cb: {e}')

    def human_move_cb(self, msg: String):
        """Handle vision-confirmed human move: update piece_map and sync Gazebo.

        If the GUI already teleported this piece via gui_move_cb, skip the
        re-teleport and map update — just acknowledge and let the engine respond.
        """
        uci = msg.data.strip()
        self.get_logger().info(f'Human move confirmed by vision: {uci}')

        if uci in self._pending_gui_moves:
            self._pending_gui_moves.discard(uci)
            self.get_logger().info(
                f'GUI move confirmed: {uci} — piece_map already updated, skipping re-teleport')
            return
        try:
            move      = chess.Move.from_uci(uci)
            from_name = chess.square_name(move.from_square)
            to_name   = chess.square_name(move.to_square)

            # Fetch both models before any teleports to avoid stale-map double-teleport
            cap_model    = self._piece_map.get(to_name)
            moving_model = self._piece_map.get(from_name)

            if not moving_model:
                self.get_logger().warn(
                    f'human_move_cb: no model for {from_name} — piece_map may be stale')

            # Teleport captured piece to graveyard.
            # Guard: cap_model must belong to the *opponent* of the mover so that
            # a stale second fire (after gui_move_cb already updated the map) cannot
            # graveyard the white piece that now sits on to_name.
            mover_is_white = moving_model is not None and moving_model.startswith('wp_')
            cap_is_opponent = (cap_model is not None and
                               cap_model != moving_model and
                               (mover_is_white == cap_model.startswith('bp_')))
            if cap_is_opponent:
                self.get_logger().info(f'Capture: {cap_model} on {to_name} → graveyard')
                self._teleport_to_graveyard(cap_model)
            elif cap_model and not cap_is_opponent:
                self.get_logger().warn(
                    f'human_move_cb: skipped cap_model={cap_model} '
                    f'(same color or same model as mover={moving_model})')

            # Teleport moving piece to destination
            if moving_model:
                px, py, _ = self.square_to_xyz(move.to_square)
                self._teleport(moving_model, px, py, self._piece_z(moving_model))
                self.get_logger().info(
                    f'Teleport: {moving_model}  {from_name} → {to_name}  ({px:.3f}, {py:.3f})')

            self._apply_move_to_map(move)
            self.get_logger().info(
                f'Human move applied: {uci}  piece_map={len(self._piece_map)}')

        except Exception as e:
            self.get_logger().warn(f'human_move_cb: {e}')

    def _centroids_cb(self, msg: String):
        """Cache latest per-square world XY centroids from the vision node."""
        try:
            self._piece_centroids = json.loads(msg.data)
            self.get_logger().debug(
                f'[CENTROIDS] received {len(self._piece_centroids)} centroids')
        except Exception as e:
            self.get_logger().warn(f'_centroids_cb: {e}')

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = ChessArmNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
