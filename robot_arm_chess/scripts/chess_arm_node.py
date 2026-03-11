#!/usr/bin/env python3
"""
chess_arm_node.py
Converts chess moves (UCI format) into pick-and-place arm trajectories.
Uses the analytical IK from arm_ik.py to send goals to arm_controller.

Flow:
  engine_move (UCI) → compute from/to XYZ → pick piece → place piece

Subscribes:
  /chess/engine_move   (std_msgs/String) — arm's move in UCI (e.g. e7e5)
  /chess/board_state   (std_msgs/String) — FEN for capture detection

Publishes:
  /chess/arm_move      (std_msgs/String) — confirms move executed
  /chess/arm_status    (std_msgs/String) — IDLE / MOVING / DONE / ERROR
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import chess
import threading
import time


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

        self.ox      = self.get_parameter('origin_x').value
        self.oy      = self.get_parameter('origin_y').value
        self.oz      = self.get_parameter('origin_z').value
        self.sq      = self.get_parameter('square_size').value
        self.z_grasp = self.get_parameter('grasp_height').value
        self.z_lift  = self.get_parameter('lift_height').value
        self.z_hover = self.get_parameter('hover_height').value
        self.g_open  = self.get_parameter('gripper_open').value
        self.g_close = self.get_parameter('gripper_closed').value
        self.dur     = self.get_parameter('move_duration').value

        self.board = chess.Board()
        self.busy  = False

        # Publishers
        self.arm_pub     = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.confirm_pub = self.create_publisher(String, '/chess/arm_move', 10)
        self.status_pub  = self.create_publisher(String, '/chess/arm_status', 10)

        # Subscribers
        self.move_sub = self.create_subscription(
            String, '/chess/engine_move', self.engine_move_cb, 10)
        self.board_sub = self.create_subscription(
            String, '/chess/board_state', self.board_state_cb, 10)

        self.publish_status('IDLE')
        self.get_logger().info('Chess arm node ready')

    # ── Board geometry ─────────────────────────────────────────────────────────
    def square_to_xyz(self, square: chess.Square, z_offset: float = 0.0):
        """Convert chess square index to world XYZ coordinates."""
        file = chess.square_file(square)  # 0=a ... 7=h
        rank = chess.square_rank(square)  # 0=1 ... 7=8
        x = self.ox + rank * self.sq
        y = self.oy + file * self.sq
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
        """Pick up piece from square."""
        self.get_logger().info(f'Picking from {chess.square_name(square)}')

        # Open gripper
        self.send_gripper(self.g_open)

        # Hover above square
        x, y, z = self.square_to_xyz(square, self.z_hover)
        self.move_to_xyz(x, y, z)

        # Lower to grasp height
        x, y, z = self.square_to_xyz(square, self.z_grasp)
        self.move_to_xyz(x, y, z, duration=1.5)

        # Close gripper
        self.send_gripper(self.g_close)

        # Lift piece
        x, y, z = self.square_to_xyz(square, self.z_lift)
        self.move_to_xyz(x, y, z, duration=1.5)

    def place_piece(self, square: chess.Square):
        """Place piece on square."""
        self.get_logger().info(f'Placing on {chess.square_name(square)}')

        # Hover above target square
        x, y, z = self.square_to_xyz(square, self.z_hover)
        self.move_to_xyz(x, y, z)

        # Lower to place height
        x, y, z = self.square_to_xyz(square, self.z_grasp)
        self.move_to_xyz(x, y, z, duration=1.5)

        # Release
        self.send_gripper(self.g_open)

        # Lift away
        x, y, z = self.square_to_xyz(square, self.z_lift)
        self.move_to_xyz(x, y, z, duration=1.5)

    def remove_captured_piece(self, square: chess.Square):
        """Move captured piece off the board."""
        self.get_logger().info(f'Removing captured piece from {chess.square_name(square)}')
        self.pick_piece(square)
        # Move to off-board dump zone
        self.move_to_xyz(self.ox - 0.10, self.oy - 0.05, self.z_hover)
        self.send_gripper(self.g_open)
        self.move_to_xyz(self.ox - 0.10, self.oy - 0.05, self.z_lift)

    def return_home(self):
        """Return arm to home position."""
        home = {'base_yaw': 0.0, 'shoulder_roll': 0.0,
                'shoulder_pitch': 0.0, 'elbow_pitch': 0.0}
        self.send_arm(home, duration=2.0)

    # ── Main move execution ───────────────────────────────────────────────────
    def execute_move(self, uci: str):
        self.publish_status('MOVING')
        try:
            move = chess.Move.from_uci(uci)
            from_sq = move.from_square
            to_sq   = move.to_square

            # Handle capture — remove opponent piece first
            if self.board.is_capture(move):
                self.get_logger().info('Capture detected — removing piece first')
                self.remove_captured_piece(to_sq)

            # Pick and place
            self.pick_piece(from_sq)
            self.place_piece(to_sq)

            # Return home
            self.return_home()

            # Confirm move
            msg = String()
            msg.data = uci
            self.confirm_pub.publish(msg)
            self.publish_status('DONE')
            self.get_logger().info(f'Move {uci} executed successfully')

        except Exception as e:
            self.get_logger().error(f'Move execution failed: {e}')
            self.publish_status('ERROR')
        finally:
            self.busy = False

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
