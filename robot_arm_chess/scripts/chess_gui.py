#!/usr/bin/env python3
"""
chess_gui.py
Terminal + tkinter GUI for the human player.
Displays board state, accepts human move input, shows arm status.

Publishes:
  /chess/gui_move     (std_msgs/String) — GUI-submitted move: Gazebo teleport only.
                                          chess_vision_node detects the teleport and
                                          publishes /chess/human_move to trigger the game.

Subscribes:
  /chess/board_state  (std_msgs/String) — FEN string
  /chess/arm_status   (std_msgs/String) — IDLE / MOVING / DONE / ERROR
  /chess/game_status  (std_msgs/String) — game result
  /chess/engine_move  (std_msgs/String) — arm's last move
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import chess
import tkinter as tk
from tkinter import font as tkfont
import threading


LIGHT = '#F0D9B5'
DARK  = '#B58863'
HIGHLIGHT = '#AAD751'
SELECTED  = '#F6F669'
PIECE_UNICODE = {
    chess.Piece(chess.KING,   chess.WHITE): '♔',
    chess.Piece(chess.QUEEN,  chess.WHITE): '♕',
    chess.Piece(chess.ROOK,   chess.WHITE): '♖',
    chess.Piece(chess.BISHOP, chess.WHITE): '♗',
    chess.Piece(chess.KNIGHT, chess.WHITE): '♘',
    chess.Piece(chess.PAWN,   chess.WHITE): '♙',
    chess.Piece(chess.KING,   chess.BLACK): '♚',
    chess.Piece(chess.QUEEN,  chess.BLACK): '♛',
    chess.Piece(chess.ROOK,   chess.BLACK): '♜',
    chess.Piece(chess.BISHOP, chess.BLACK): '♝',
    chess.Piece(chess.KNIGHT, chess.BLACK): '♞',
    chess.Piece(chess.PAWN,   chess.BLACK): '♟',
}


class ChessGUI(Node):

    def __init__(self):
        super().__init__('chess_gui')

        self.board = chess.Board()
        self.selected_square = None
        self.arm_status = 'IDLE'
        self.game_status = 'ONGOING'
        self.last_arm_move = None

        # Publishers
        self.human_pub = self.create_publisher(String, '/chess/gui_move', 10)
        self.cmd_pub   = self.create_publisher(String, '/chess/cmd',        10)

        # Subscribers
        self.create_subscription(String, '/chess/board_state',  self.board_cb,  10)
        self.create_subscription(String, '/chess/arm_status',   self.status_cb, 10)
        self.create_subscription(String, '/chess/game_status',  self.game_cb,   10)
        self.create_subscription(String, '/chess/engine_move',  self.engine_cb, 10)

        self._build_gui()

    # ── GUI ────────────────────────────────────────────────────────────────────
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title('Chess — Human vs Robot Arm')
        self.root.configure(bg='#1a1a2e')
        self.root.resizable(False, False)

        # Header
        tk.Label(self.root, text='CHESS  ·  Human vs Robot Arm',
                 bg='#1a1a2e', fg='#e0e0e0',
                 font=('Courier', 13, 'bold')).pack(pady=8)

        # Board canvas
        self.canvas = tk.Canvas(self.root, width=400, height=400,
                                bg='#1a1a2e', highlightthickness=0)
        self.canvas.pack(padx=20)
        self.canvas.bind('<Button-1>', self._on_click)

        # Status bar
        self.status_var = tk.StringVar(value='Your turn (White)')
        tk.Label(self.root, textvariable=self.status_var,
                 bg='#1a1a2e', fg='#58a6ff',
                 font=('Courier', 10)).pack(pady=4)

        # Arm status
        self.arm_var = tk.StringVar(value='Arm: IDLE')
        tk.Label(self.root, textvariable=self.arm_var,
                 bg='#1a1a2e', fg='#3fb950',
                 font=('Courier', 9)).pack()

        # Move input
        input_frame = tk.Frame(self.root, bg='#1a1a2e')
        input_frame.pack(pady=8)
        tk.Label(input_frame, text='Move (UCI):', bg='#1a1a2e',
                 fg='#8b949e', font=('Courier', 9)).pack(side='left')
        self.move_entry = tk.Entry(input_frame, width=8,
                                   bg='#21262d', fg='white',
                                   insertbackground='white',
                                   font=('Courier', 11))
        self.move_entry.pack(side='left', padx=6)
        self.move_entry.bind('<Return>', self._submit_move)
        tk.Button(input_frame, text='Send',
                  command=self._submit_move,
                  bg='#0d419d', fg='white',
                  font=('Courier', 9), bd=0, padx=8,
                  cursor='hand2').pack(side='left')

        # Game control row
        game_frame = tk.Frame(self.root, bg='#1a1a2e')
        game_frame.pack(pady=(4, 1))
        tk.Label(game_frame, text='Game:', bg='#1a1a2e', fg='#8b949e',
                 font=('Courier', 8)).pack(side='left', padx=(0, 4))
        tk.Button(game_frame, text='Reset Game',
                  command=self._reset_game,
                  bg='#5a1a1a', fg='white', font=('Courier', 9),
                  bd=0, padx=10, pady=4, cursor='hand2').pack(side='left', padx=4)
        tk.Button(game_frame, text='Return Pieces',
                  command=self._return_pieces,
                  bg='#2a2a6a', fg='white', font=('Courier', 9),
                  bd=0, padx=10, pady=4, cursor='hand2').pack(side='left', padx=4)

        # Vision calibration row
        calib_frame = tk.Frame(self.root, bg='#1a1a2e')
        calib_frame.pack(pady=(1, 4))
        tk.Label(calib_frame, text='Vision:', bg='#1a1a2e', fg='#8b949e',
                 font=('Courier', 8)).pack(side='left', padx=(0, 4))
        tk.Button(calib_frame, text='Calibrate Camera',
                  command=self._remove_pieces,
                  bg='#1a4a2a', fg='white', font=('Courier', 9),
                  bd=0, padx=10, pady=4, cursor='hand2').pack(side='left', padx=4)
        tk.Button(calib_frame, text='Recalibrate',
                  command=self._recalibrate,
                  bg='#3a3a1a', fg='white', font=('Courier', 9),
                  bd=0, padx=10, pady=4, cursor='hand2').pack(side='left', padx=4)

        # Last moves log
        self.log_text = tk.Text(self.root, width=44, height=6,
                                bg='#0d1117', fg='#8b949e',
                                font=('Courier', 8), state='disabled',
                                relief='flat')
        self.log_text.pack(padx=20, pady=6)

        self._draw_board()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _draw_board(self):
        self.canvas.delete('all')
        sq_size = 50
        piece_font = tkfont.Font(family='Segoe UI Symbol', size=28)

        for rank in range(7, -1, -1):
            for file in range(8):
                x1 = file * sq_size
                y1 = (7 - rank) * sq_size
                x2 = x1 + sq_size
                y2 = y1 + sq_size

                sq = chess.square(file, rank)
                color = LIGHT if (file + rank) % 2 == 0 else DARK
                if sq == self.selected_square:
                    color = SELECTED
                elif self.last_arm_move:
                    move = chess.Move.from_uci(self.last_arm_move)
                    if sq in [move.from_square, move.to_square]:
                        color = HIGHLIGHT

                self.canvas.create_rectangle(x1, y1, x2, y2,
                                             fill=color, outline='')

                piece = self.board.piece_at(sq)
                if piece:
                    symbol = PIECE_UNICODE.get(piece, '?')
                    self.canvas.create_text(
                        x1 + sq_size // 2, y1 + sq_size // 2,
                        text=symbol, font=piece_font,
                        fill='#1a1a2e' if piece.color == chess.WHITE else '#f0f0f0'
                    )

        # File labels
        for file in range(8):
            self.canvas.create_text(
                file * sq_size + sq_size // 2, 395,
                text=chess.FILE_NAMES[file],
                fill='#8b949e', font=('Courier', 8))

    def _on_click(self, event):
        if self.board.turn != chess.WHITE:
            return
        sq_size = 50
        file = event.x // sq_size
        rank = 7 - (event.y // sq_size)
        if not (0 <= file <= 7 and 0 <= rank <= 7):
            return
        sq = chess.square(file, rank)

        if self.selected_square is None:
            if self.board.piece_at(sq) and \
               self.board.piece_at(sq).color == chess.WHITE:
                self.selected_square = sq
        else:
            uci = chess.square_name(self.selected_square) + chess.square_name(sq)
            self.selected_square = None
            self._send_human_move(uci)

        self._draw_board()

    def _submit_move(self, event=None):
        uci = self.move_entry.get().strip()
        self.move_entry.delete(0, tk.END)
        if uci:
            self._send_human_move(uci)

    def _send_human_move(self, uci: str):
        try:
            move = chess.Move.from_uci(uci)
            if move in self.board.legal_moves:
                msg = String()
                msg.data = uci
                self.human_pub.publish(msg)
                self._log(f'You: {uci}')
                # Immediately update local board so the canvas redraws at once.
                # The authoritative state syncs back via /chess/board_state once
                # vision confirms the physical move.
                self.board.push(move)
                self.root.after(0, self._draw_board)
                self.root.after(0, lambda: self.status_var.set('Waiting for camera...'))
            else:
                self._log(f'Illegal: {uci}')
        except ValueError:
            self._log(f'Invalid: {uci}')

    def _reset_game(self):
        msg = String()
        msg.data = 'RESET'
        self.cmd_pub.publish(msg)
        self.board = chess.Board()
        self.last_arm_move = None
        self._log('--- Game reset ---')
        self.root.after(0, self._draw_board)

    def _return_pieces(self):
        msg = String()
        msg.data = 'RETURN_PIECES'
        self.cmd_pub.publish(msg)
        self._log('Returning pieces to current board positions...')

    def _remove_pieces(self):
        msg = String()
        msg.data = 'REMOVE_PIECES'
        self.cmd_pub.publish(msg)
        self._log('Removing pieces for camera calibration...')

    def _recalibrate(self):
        msg = String()
        msg.data = 'RECAL'
        self.cmd_pub.publish(msg)
        self._log('Recalibrating empty board reference...')

    def _log(self, text: str):
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, text + '\n')
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')

    # ── ROS callbacks (thread-safe via after()) ────────────────────────────────
    def board_cb(self, msg: String):
        try:
            self.board = chess.Board(msg.data)
            self.root.after(0, self._draw_board)
            turn = 'Your turn (White)' if self.board.turn == chess.WHITE \
                   else 'Arm thinking...'
            self.root.after(0, lambda: self.status_var.set(turn))
        except Exception:
            pass

    def status_cb(self, msg: String):
        self.arm_status = msg.data
        self.root.after(0, lambda: self.arm_var.set(f'Arm: {msg.data}'))
        if msg.data == 'DONE':
            self.root.after(0, lambda: self.status_var.set('Your turn (White)'))

    def game_cb(self, msg: String):
        if msg.data not in ('ONGOING', 'CHECK'):
            self.root.after(0, lambda: self.status_var.set(msg.data))
            self._log(f'Game: {msg.data}')

    def engine_cb(self, msg: String):
        self.last_arm_move = msg.data
        self._log(f'Arm: {msg.data}')
        self.root.after(0, self._draw_board)

    def _on_close(self):
        self.root.destroy()
        rclpy.shutdown()

    def run(self):
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()
        self.root.mainloop()


def main():
    rclpy.init()
    gui = ChessGUI()
    gui.run()


if __name__ == '__main__':
    main()
