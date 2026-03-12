#!/usr/bin/env python3
"""
board_state_node.py
Tracks the chess board state and publishes it as a ROS 2 topic.
Maintains piece positions, validates moves, and updates after each move.

Publishes:
  /chess/board_state  (std_msgs/String) — FEN string of current position
  /chess/last_move    (std_msgs/String) — last move in UCI format (e.g. e2e4)

Subscribes:
  /chess/human_move   (std_msgs/String) — human move in UCI format
  /chess/arm_move     (std_msgs/String) — arm move in UCI format
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import chess


class BoardStateNode(Node):

    def __init__(self):
        super().__init__('board_state_node')

        # Load params
        self.declare_parameter('arm_plays_as', 'black')
        self.arm_color = self.get_parameter('arm_plays_as').value

        # Chess board
        self.board = chess.Board()
        self.get_logger().info('Chess board initialized — starting position')
        self.get_logger().info(f'Arm plays as: {self.arm_color}')

        # Publishers
        self.board_pub = self.create_publisher(String, '/chess/board_state', 10)
        self.last_move_pub = self.create_publisher(String, '/chess/last_move', 10)
        self.status_pub = self.create_publisher(String, '/chess/game_status', 10)

        # Subscribers
        self.human_sub = self.create_subscription(
            String, '/chess/human_move', self.human_move_cb, 10)
        self.arm_sub = self.create_subscription(
            String, '/chess/arm_move', self.arm_move_cb, 10)
        self.cmd_sub = self.create_subscription(
            String, '/chess/cmd', self.cmd_cb, 10)

        # Publish initial state
        self.publish_state()
        self.print_board()

    def cmd_cb(self, msg: String):
        if msg.data == 'RESET':
            self.board = chess.Board()
            self.publish_state()
            self.print_board()
            self.get_logger().info('Board reset to starting position')

    def human_move_cb(self, msg: String):
        uci = msg.data.strip()
        self.get_logger().info(f'Human move received: {uci}')
        self.apply_move(uci, player='human')

    def arm_move_cb(self, msg: String):
        uci = msg.data.strip()
        self.get_logger().info(f'Arm move received: {uci}')
        self.apply_move(uci, player='arm')

    def apply_move(self, uci: str, player: str):
        try:
            move = chess.Move.from_uci(uci)
        except ValueError:
            self.get_logger().error(f'Invalid UCI move format: {uci}')
            return

        if move not in self.board.legal_moves:
            self.get_logger().error(f'Illegal move: {uci}')
            self.get_logger().info(f'Legal moves: {[m.uci() for m in self.board.legal_moves]}')
            return

        # Check if it's a capture (for arm to know to remove piece first)
        is_capture = self.board.is_capture(move)
        is_castling = self.board.is_castling(move)
        is_promotion = move.promotion is not None

        self.board.push(move)

        self.get_logger().info(
            f'{player.upper()} played: {uci}'
            + (' [capture]' if is_capture else '')
            + (' [castling]' if is_castling else '')
            + (' [promotion]' if is_promotion else '')
        )

        # Publish
        last_move_msg = String()
        last_move_msg.data = uci
        self.last_move_pub.publish(last_move_msg)
        self.publish_state()
        self.print_board()
        self.check_game_over()

    def publish_state(self):
        msg = String()
        msg.data = self.board.fen()
        self.board_pub.publish(msg)

    def check_game_over(self):
        msg = String()
        if self.board.is_checkmate():
            winner = 'Black' if self.board.turn == chess.WHITE else 'White'
            msg.data = f'CHECKMATE — {winner} wins!'
            self.get_logger().info(msg.data)
        elif self.board.is_stalemate():
            msg.data = 'STALEMATE — Draw!'
            self.get_logger().info(msg.data)
        elif self.board.is_check():
            msg.data = 'CHECK'
            self.get_logger().info('Check!')
        elif self.board.is_insufficient_material():
            msg.data = 'DRAW — Insufficient material'
        else:
            msg.data = 'ONGOING'
        self.status_pub.publish(msg)

    def print_board(self):
        self.get_logger().info(f'\n{self.board}\nFEN: {self.board.fen()}')


def main():
    rclpy.init()
    node = BoardStateNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
