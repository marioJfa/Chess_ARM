#!/usr/bin/env python3
"""
chess_arm_node.py
Converts chess moves (UCI format) into pick-and-place arm trajectories.
Uses analytical IK to send goals to arm_controller.
Teleports Gazebo piece models via gz service to match arm motion.

Subscribes:
  /chess/engine_move   (std_msgs/String) — arm's move in UCI (e.g. e7e5)
  /chess/board_state   (std_msgs/String) — FEN for capture detection
  /chess/human_move    (std_msgs/String) — human move, teleport pieces in sim

Publishes:
  /chess/arm_move      (std_msgs/String) — confirms move executed
  /chess/arm_status    (std_msgs/String) — IDLE / MOVING / DONE / ERROR
"""

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


# ── Arm geometry (must match URDF) ────────────────────────────────────────────
L1 = 0.04    # base → shoulder_roll
L2 = 0.20    # upper arm
L3 = 0.18    # forearm
L4 = 0.11    # wrist → tool0

ARM_JOINTS = ['base_yaw', 'shoulder_roll', 'shoulder_pitch', 'elbow_pitch']
GRIPPER_JOINTS = ['finger_1_joint', 'finger_2_joint', 'finger_3_joint']

JOINT_LIMITS = {
    'base_yaw':       (-math.pi,   math.pi),
    'shoulder_roll':  (-0.7854,    0.7854),
    'shoulder_pitch': (-1.5708,    2.3562),
    'elbow_pitch':    (-2.0944,    2.0944),
}

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
        self.declare_parameter('grasp_height',  0.04)
        self.declare_parameter('lift_height',   0.20)
        self.declare_parameter('hover_height',  0.12)
        self.declare_parameter('gripper_open',  0.0)
        self.declare_parameter('gripper_closed',1.05)
        self.declare_parameter('move_duration', 2.5)
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
        self.z_grasp    = self.get_parameter('grasp_height').value
        self.z_lift     = self.get_parameter('lift_height').value
        self.z_hover    = self.get_parameter('hover_height').value
        self.g_open     = self.get_parameter('gripper_open').value
        self.g_close    = self.get_parameter('gripper_closed').value
        self.dur        = self.get_parameter('move_duration').value

        self.sb_base_yaw       = self.get_parameter('standby_base_yaw').value
        self.sb_shoulder_roll  = self.get_parameter('standby_shoulder_roll').value
        self.sb_shoulder_pitch = self.get_parameter('standby_shoulder_pitch').value
        self.sb_elbow_pitch    = self.get_parameter('standby_elbow_pitch').value

        self.add_on_set_parameters_callback(self._on_param_change)

        self.board = chess.Board()
        self.busy  = False

        # Piece tracking: square_name -> gz model name  e.g. 'e2' -> 'wp_pe2'
        self._piece_map  = {}
        self._grave_idx  = 0
        self._init_piece_map()

        self._latest_joint_states = None   # updated by /joint_states subscriber

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
        self.cmd_sub = self.create_subscription(
            String, '/chess/cmd', self.cmd_cb, 10)

        self.publish_status('IDLE')
        self.get_logger().info('Chess arm node ready')

        # Move to standby once controllers are fully loaded (~6s after node start)
        self._standby_timer = self.create_timer(6.0, self._go_standby_once)

    # ── Live param updates ────────────────────────────────────────────────────
    def _on_param_change(self, params):
        for p in params:
            n, v = p.name, p.value
            if n == 'origin_x':            self.ox         = v
            elif n == 'origin_y':          self.oy         = v
            elif n == 'origin_z':          self.oz         = v
            elif n == 'square_size':       self.sq         = v
            elif n == 'board_flip':        self.board_flip = v
            elif n == 'grasp_height':      self.z_grasp    = v
            elif n == 'lift_height':       self.z_lift     = v
            elif n == 'hover_height':      self.z_hover    = v
            elif n == 'gripper_open':      self.g_open     = v
            elif n == 'gripper_closed':    self.g_close    = v
            elif n == 'move_duration':     self.dur        = v
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
            # Calibration: remove pieces so vision can capture a clean empty-board reference
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
        """Teleport all 32 pieces back to their starting squares."""
        files = 'abcdefgh'
        back  = ['r', 'n', 'b', 'q', 'k', 'b', 'n', 'r']
        for i, p in enumerate(back):
            sq = chess.square(i, 0)
            x, y, _ = self.square_to_xyz(sq)
            self._teleport(f'wp_{p}{files[i]}1', x, y, PIECE_Z)
            sq = chess.square(i, 7)
            x, y, _ = self.square_to_xyz(sq)
            self._teleport(f'bp_{p}{files[i]}8', x, y, PIECE_Z)
        for i in range(8):
            sq = chess.square(i, 1)
            x, y, _ = self.square_to_xyz(sq)
            self._teleport(f'wp_p{files[i]}2', x, y, PAWN_Z)
            sq = chess.square(i, 6)
            x, y, _ = self.square_to_xyz(sq)
            self._teleport(f'bp_p{files[i]}7', x, y, PAWN_Z)
        self.board = chess.Board()
        self._grave_idx = 0
        self._init_piece_map()
        self.busy = False
        self.publish_status('IDLE')
        self.get_logger().info('All pieces reset to starting positions')

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
    def square_to_xyz(self, square: chess.Square, z_offset: float = 0.0):
        file = chess.square_file(square)
        rank = chess.square_rank(square)
        if self.board_flip:
            x = self.ox + (7 - rank) * self.sq + self.sq / 2
            y = self.oy + (7 - file) * self.sq + self.sq / 2
        else:
            x = self.ox + rank * self.sq + self.sq / 2
            y = self.oy + file * self.sq + self.sq / 2
        z = self.oz + z_offset
        return x, y, z

    # ── IK solver ─────────────────────────────────────────────────────────────
    def analytical_ik(self, x, y, z):
        base_yaw = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2)
        shoulder_z = 0.06 + L1
        z_eff = z - shoulder_z
        D = math.sqrt(r**2 + z_eff**2)

        reach_max = L2 + L3 + L4
        if D > reach_max:
            self.get_logger().warn(
                f'Target unreachable: {D:.3f}m > {reach_max:.3f}m, clamping')
            D = reach_max * 0.95

        L34 = L3 + L4
        cos_elbow = (L2**2 + L34**2 - D**2) / (2 * L2 * L34)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_pitch = math.pi - math.acos(cos_elbow)

        alpha = math.atan2(z_eff, r)
        cos_beta = (L2**2 + D**2 - L34**2) / (2 * L2 * D)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        shoulder_total = alpha + beta

        solution = {
            'base_yaw':       base_yaw,
            'shoulder_roll':  shoulder_total * 0.4,
            'shoulder_pitch': shoulder_total * 0.6,
            'elbow_pitch':    elbow_pitch,
        }
        for j, v in solution.items():
            lo, hi = JOINT_LIMITS[j]
            solution[j] = max(lo, min(hi, v))
        return solution

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
        sol = self.analytical_ik(x, y, z)
        self.send_arm(sol, duration)

    # ── Pick and place ────────────────────────────────────────────────────────
    def pick_piece(self, square: chess.Square):
        self.get_logger().info(f'Picking from {chess.square_name(square)}')
        self.send_gripper(self.g_open)
        x, y, z = self.square_to_xyz(square, self.z_hover)
        self.move_to_xyz(x, y, z)
        x, y, z = self.square_to_xyz(square, self.z_grasp)
        self.move_to_xyz(x, y, z, duration=1.5)
        self.send_gripper(self.g_close)
        x, y, z = self.square_to_xyz(square, self.z_lift)
        self.move_to_xyz(x, y, z, duration=1.5)

    def place_piece(self, square: chess.Square, model_name: str = None):
        self.get_logger().info(f'Placing on {chess.square_name(square)}')
        x, y, z = self.square_to_xyz(square, self.z_hover)
        self.move_to_xyz(x, y, z)
        x, y, z = self.square_to_xyz(square, self.z_grasp)
        self.move_to_xyz(x, y, z, duration=1.5)
        # Teleport piece to destination as the arm lowers to place it
        if model_name:
            px, py, _ = self.square_to_xyz(square)
            self._teleport(model_name, px, py, self._piece_z(model_name))
        self.send_gripper(self.g_open)
        x, y, z = self.square_to_xyz(square, self.z_lift)
        self.move_to_xyz(x, y, z, duration=1.5)

    def remove_captured_piece(self, square: chess.Square, model_name: str = None):
        self.get_logger().info(f'Removing captured piece from {chess.square_name(square)}')
        if model_name:
            self._teleport_to_graveyard(model_name)
        self.pick_piece(square)
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

    # ── Main move execution ───────────────────────────────────────────────────
    def execute_move(self, uci: str):
        self.publish_status('MOVING')
        try:
            move     = chess.Move.from_uci(uci)
            from_sq  = move.from_square
            to_sq    = move.to_square
            from_name = chess.square_name(from_sq)
            to_name   = chess.square_name(to_sq)

            # Handle capture — teleport captured piece to graveyard + arm animation
            if self.board.is_capture(move):
                self.get_logger().info('Capture — removing piece first')
                cap_model = self._piece_map.get(to_name)
                self.remove_captured_piece(to_sq, cap_model)

            # Get the model being moved
            moving_model = self._piece_map.get(from_name)

            # Pick and place (piece teleports to destination inside place_piece)
            self.pick_piece(from_sq)
            self.place_piece(to_sq, moving_model)

            # Update internal map
            self._apply_move_to_map(move)

            self.return_home()
            self._wait_for_arm_stop()   # arm physically stopped → safe to sample

            msg = String()
            msg.data = uci
            self.confirm_pub.publish(msg)
            self.publish_status('DONE')
            self.get_logger().info(f'Move {uci} executed')

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

    def human_move_cb(self, msg: String):
        """Teleport human's piece in Gazebo to keep sim in sync."""
        uci = msg.data.strip()
        try:
            move      = chess.Move.from_uci(uci)
            from_name = chess.square_name(move.from_square)
            to_name   = chess.square_name(move.to_square)

            # Teleport captured piece to graveyard
            cap_model = self._piece_map.get(to_name)
            if cap_model:
                self._teleport_to_graveyard(cap_model)

            # Teleport moving piece to destination
            model = self._piece_map.get(from_name)
            if model:
                px, py, _ = self.square_to_xyz(move.to_square)
                self._teleport(model, px, py, self._piece_z(model))

            self._apply_move_to_map(move)

        except Exception as e:
            self.get_logger().warn(f'human_move_cb: {e}')

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
