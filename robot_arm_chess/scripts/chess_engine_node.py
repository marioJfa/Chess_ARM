#!/usr/bin/env python3
"""
chess_engine_node.py
Interfaces with Stockfish to calculate the best move.
Listens for board state changes, calculates response, publishes move.

Subscribes:
  /chess/board_state   (std_msgs/String) — FEN string
  /chess/game_status   (std_msgs/String) — ONGOING / CHECKMATE / etc.

Publishes:
  /chess/engine_move   (std_msgs/String) — best move in UCI format
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import chess
import chess.engine
import threading
import os


class ChessEngineNode(Node):

    def __init__(self):
        super().__init__('chess_engine_node')

        # Params
        self.declare_parameter('stockfish_path', '/usr/games/stockfish')
        self.declare_parameter('depth', 10)
        self.declare_parameter('move_time_ms', 2000)
        self.declare_parameter('skill_level', 10)
        self.declare_parameter('arm_plays_as', 'black')

        stockfish_path = self.get_parameter('stockfish_path').value
        self.depth = self.get_parameter('depth').value
        self.move_time = self.get_parameter('move_time_ms').value / 1000.0
        self.skill = self.get_parameter('skill_level').value
        self.arm_color_str = self.get_parameter('arm_plays_as').value
        self.arm_color = chess.BLACK if self.arm_color_str == 'black' else chess.WHITE

        # Verify stockfish exists
        if not os.path.exists(stockfish_path):
            # Try alternate paths
            for path in ['/usr/bin/stockfish', '/usr/local/bin/stockfish']:
                if os.path.exists(path):
                    stockfish_path = path
                    break
            else:
                self.get_logger().error(
                    f'Stockfish not found at {stockfish_path}. '
                    'Install with: sudo apt install stockfish')
                return

        # Start Stockfish
        self.engine = chess.engine.SimpleEngine.popen_uci(stockfish_path)
        self.engine.configure({'Skill Level': self.skill})
        self.get_logger().info(
            f'Stockfish loaded — depth={self.depth} skill={self.skill} '
            f'arm_plays={self.arm_color_str}')

        self.board = chess.Board()
        self.game_active = True
        self.calculating = False

        # Publishers
        self.move_pub = self.create_publisher(String, '/chess/engine_move', 10)

        # Subscribers
        self.board_sub = self.create_subscription(
            String, '/chess/board_state', self.board_state_cb, 10)
        self.status_sub = self.create_subscription(
            String, '/chess/game_status', self.status_cb, 10)
        self.cmd_sub = self.create_subscription(
            String, '/chess/cmd', self.cmd_cb, 10)

    def cmd_cb(self, msg: String):
        if msg.data.strip() == 'RESET':
            self.board = chess.Board()
            self.game_active = True
            self.calculating = False
            self.get_logger().info('Engine reset — game_active=True, ready for new game')

    def status_cb(self, msg: String):
        if msg.data not in ('ONGOING', 'CHECK'):
            self.game_active = False
            self.get_logger().info(f'Game over: {msg.data}')

    def board_state_cb(self, msg: String):
        if not self.game_active or self.calculating:
            return

        try:
            self.board = chess.Board(msg.data)
        except ValueError as e:
            self.get_logger().error(f'Invalid FEN: {e}')
            return

        # Only calculate if it's the arm's turn
        if self.board.turn != self.arm_color:
            return

        # Calculate in background thread so we don't block ROS spin
        thread = threading.Thread(target=self.calculate_move, daemon=True)
        thread.start()

    def calculate_move(self):
        self.calculating = True
        self.get_logger().info('Calculating best move...')

        try:
            result = self.engine.play(
                self.board,
                chess.engine.Limit(depth=self.depth, time=self.move_time)
            )
            move = result.move
            self.get_logger().info(f'Best move: {move.uci()}')

            msg = String()
            msg.data = move.uci()
            self.move_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Engine error: {e}')
        finally:
            self.calculating = False

    def destroy_node(self):
        if hasattr(self, 'engine'):
            self.engine.quit()
        super().destroy_node()


def main():
    rclpy.init()
    node = ChessEngineNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
