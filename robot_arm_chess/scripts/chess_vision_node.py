#!/usr/bin/env python3
"""
chess_vision_node.py — Board-first piece tracking.

Pipeline:
  SEARCHING     → Hough line detection on tile edges each frame (arm must be idle)
  WAIT_EMPTY    → board found, waiting for "Calibrate Camera" command
  CAPTURING_REF → collecting empty-board reference frames
  TRACKING      → frame-diff vs empty ref to detect pieces + movement

Publishes:
  /chess/vision/white_squares  — JSON list of squares with white pieces
  /chess/vision/debug_image    — annotated image
  /chess/human_move            — detected human move in UCI format (e.g. "e2e4")

Subscribes:
  /camera/image_raw
  /camera/camera_info
  /chess/arm_status
  /chess/last_move
  /chess/cmd
"""

import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from cv_bridge import CvBridge

FILES = 'abcdefgh'

# The 32 squares that are occupied at the start of any chess game (ranks 1,2,7,8)
STARTING_SQUARES = frozenset(
    f'{f}{r}' for f in 'abcdefgh' for r in (1, 2, 7, 8)
)

# ── ArUco board reference ──────────────────────────────────────────────────────
# Four markers (IDs 0-3) placed at the outer corners of the board.
# Physical placement (viewed from white's side):
#   ID 0 → a1 corner  (bottom-left)
#   ID 1 → h1 corner  (bottom-right)
#   ID 2 → a8 corner  (top-left)
#   ID 3 → h8 corner  (top-right)
#
# ArUco corner order per marker: [top-left, top-right, bottom-right, bottom-left]
# "Inner corner" = the corner of the marker that touches the board playing area.
ARUCO_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()   # OpenCV 4.6 API

# Index into marker.corners[0] that gives the inner (board-facing) corner
MARKER_INNER = {0: 1, 1: 0, 2: 2, 3: 3}

# Board-space position (in squares) of each inner corner — a1=(0,0), h8=(8,8)
# x increases rank1→rank8,  y increases a-file→h-file
# Verified from chess_world.sdf geometry: board at world (0.3575, -0.0175),
# sq_a1 local (-0.135,-0.135), marker centres at (0.1555, ±0.2195/0.2295).
# Gap from board edge to inner corner = 0.0145 m → 0.0145/0.045 = 0.322 squares.
_D = 0.322   # inner-corner offset from board playing-area edge in squares

# Marker physical size = 0.060m (chess_world.sdf), square size = 0.045m
# → MARKER_SQ = 0.060/0.045 = 4/3 board squares exactly.
MARKER_SQ = 4.0 / 3.0

# Inner-corner board-space positions (kept for reference; _detect_via_aruco uses
# live self.aruco_offset and builds ALL 16 corner positions dynamically)
MARKER_BOARD = {
    0: (-_D,       -_D),
    1: (8.0 + _D,  -_D),
    2: (-_D,       8.0 + _D),
    3: (8.0 + _D,  8.0 + _D),
}

BOARD_SCALE = 100   # virtual pixels per square used for homography arithmetic
MOVE_STABLE_FRAMES  = 15   # consecutive stable frames before publishing detected move
MARKER_STABLE_FRAMES = 8   # frames ArUco centroid must be still → arm truly stopped
MARKER_STABLE_THR    = 2.0 # max pixel drift per frame to be considered stable


class ChessVisionNode(Node):
    # States
    SEARCHING     = 'SEARCHING'      # trying to find board grid
    WAIT_EMPTY    = 'WAIT_EMPTY'     # board found, waiting for "Calibrate Camera"
    CAPTURING_REF = 'CAPTURING_REF'  # collecting empty-board reference frames
    WAIT_PIECES   = 'WAIT_PIECES'    # ref done, waiting for all 32 pieces to be placed
    TRACKING      = 'TRACKING'       # live piece detection + move tracking (diff vs ref + prev)

    REF_FRAMES_NEEDED = 15           # frames to average for empty-board ref

    def __init__(self):
        super().__init__('chess_vision_node')

        self.declare_parameter('origin_x',         0.20)
        self.declare_parameter('origin_y',         -0.175)
        self.declare_parameter('origin_z',          0.02)
        self.declare_parameter('square_size',       0.045)
        self.declare_parameter('sample_radius',       10)
        self.declare_parameter('change_threshold',    18.0)
        self.declare_parameter('piece_threshold',     22.0)
        self.declare_parameter('debug_diff',          False)  # show per-square diff overlay
        self.declare_parameter('aruco_inner_offset',  0.322)    # _D: marker inner corner → board edge (squares)
        self.declare_parameter('marker_sq_size',      MARKER_SQ)  # marker side in board squares (0.060/0.045)
        self.declare_parameter('grid_dx',             0)      # pixel nudge right (+ ) / left  (-)
        self.declare_parameter('grid_dy',             0)      # pixel nudge down  (+ ) / up    (-)

        self.ox               = self.get_parameter('origin_x').value
        self.oy               = self.get_parameter('origin_y').value
        self.oz               = self.get_parameter('origin_z').value
        self.sq               = self.get_parameter('square_size').value
        self.sample_r         = self.get_parameter('sample_radius').value
        self.change_thr       = self.get_parameter('change_threshold').value
        self.piece_thr        = self.get_parameter('piece_threshold').value
        self.debug_diff       = self.get_parameter('debug_diff').value
        self.aruco_offset     = self.get_parameter('aruco_inner_offset').value
        self.marker_sq        = self.get_parameter('marker_sq_size').value
        self.grid_dx          = self.get_parameter('grid_dx').value
        self.grid_dy          = self.get_parameter('grid_dy').value

        self.add_on_set_parameters_callback(self._on_param_change)

        self.bridge           = CvBridge()
        self.camera_info      = None
        self.arm_idle         = False  # False until first IDLE/DONE message received
        self._arm_just_idled  = False  # True for one detection cycle on MOVING→IDLE edge
        self.last_move           = None
        self._last_det_method    = ''     # which detector last found the board

        # Vision state
        self.state          = self.SEARCHING
        self.homography     = None  # board-space → pixel-space (set by ArUco)
        self._last_aruco_corners   = None  # {mid: 4×2 px array} saved for live recompute
        self._last_aruco_frame_hw  = None  # (h, w) of frame when ArUco last ran
        self.grid_centres   = {}    # sq → (u, v)  — in-frame only
        self.empty_ref      = {}    # sq → mean gray brightness of empty square
        self.prev_idle_gray = {}    # sq → mean gray at last idle frame
        self.piece_sqs      = set() # squares currently occupied (diff vs empty_ref)
        self.changed_sqs    = set() # squares that changed since last idle frame

        # Game history — one entry per arm-idle snapshot in TRACKING
        self.board_history  = []    # [{'frame': int, 'move': str, 'pieces': set, 'changed': set}]

        # Ref capture accumulator
        self._ref_acc    = {}   # sq → list of brightness samples
        self._ref_count  = 0
        self._ref_delay  = 0   # frames to skip before accumulating (settle time)
        self._recal_mode = False  # True → skip WAIT_PIECES after ref capture (mid-game recal)

        # Logging throttle
        self._log_counter  = 0
        self._log_interval = 30

        self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb,   10)
        self.create_subscription(Image,      '/camera/image_raw',   self._image_cb,   1)
        self.create_subscription(String,     '/chess/arm_status',   self._status_cb, 10)
        self.create_subscription(String,     '/chess/last_move',    self._move_cb,   10)
        self.create_subscription(String,     '/chess/cmd',          self._cmd_cb,    10)

        # Human move monitoring — set up after arm idles, published once stable change detected
        self._awaiting_human_move  = False
        self._human_ref_sqs        = set()   # piece_sqs snapshot taken at start of human's turn
        self._human_ref_gray       = {}      # brightness snapshot at start of human's turn
        self._prev_occ_set         = None    # prev frame cur_occupied for stability tracking
        self._move_stable_counter  = 0       # frames cur_occupied has been unchanged
        self._reset_settle         = 0       # countdown frames after RESET before auto-snapshotting
        self._wp_occupied          = set()   # last stable occupancy in WAIT_PIECES
        self._wp_missing           = set()   # last stable missing set in WAIT_PIECES
        self._diff_cache           = {}      # sq → diff value, frozen when arm/cam not still

        # Marker stability — tracks ArUco centroid drift to detect arm truly stopped
        self._prev_marker_ctr      = None    # (u, v) centroid of all 16 corners last frame
        self._marker_stable_ct     = 0       # consecutive frames within MARKER_STABLE_THR
        self._markers_stable       = False   # True when arm physically stationary
        self._idle_stability_wait  = 0       # frames waited for stability after _arm_just_idled

        self.squares_pub    = self.create_publisher(String, '/chess/vision/white_squares',  10)
        self.debug_pub      = self.create_publisher(Image,  '/chess/vision/debug_image',    10)
        self.human_move_pub = self.create_publisher(String, '/chess/human_move',             10)
        self.centroids_pub  = self.create_publisher(String, '/chess/vision/piece_centroids', 10)

        self.get_logger().info('Chess vision node ready — searching for board')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _info_cb(self, msg):
        if self.camera_info is None:
            self.get_logger().info(
                f'Camera info: {msg.width}x{msg.height}  '
                f'fx={msg.k[0]:.1f} fy={msg.k[4]:.1f}  '
                f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')
        self.camera_info = msg

    def _status_cb(self, msg):
        was_idle      = self.arm_idle
        self.arm_idle = msg.data in ('IDLE', 'DONE')

        if self.arm_idle and not was_idle:
            self._arm_just_idled = True
            self.get_logger().info(
                f'Arm idle — vision detection triggered  [state={self.state}]')
        elif not self.arm_idle and was_idle:
            self._awaiting_human_move = False   # arm started moving — stop monitoring
            self.get_logger().info(f'Arm moving — vision paused  [state={self.state}]')

    def _move_cb(self, msg):
        self.last_move = msg.data.strip()
        self.get_logger().info(f'Last move: {self.last_move}')

    def _on_param_change(self, params):
        """Live parameter updates — no restart needed."""
        grid_changed = False
        for p in params:
            if p.name == 'piece_threshold':
                self.piece_thr = float(p.value)
                self.get_logger().info(f'piece_threshold → {self.piece_thr}')
            elif p.name == 'change_threshold':
                self.change_thr = float(p.value)
                self.get_logger().info(f'change_threshold → {self.change_thr}')
            elif p.name == 'sample_radius':
                self.sample_r = int(p.value)
                self.get_logger().info(f'sample_radius → {self.sample_r}')
            elif p.name == 'debug_diff':
                self.debug_diff = bool(p.value)
                self.get_logger().info(f'debug_diff → {self.debug_diff}')
            elif p.name == 'aruco_inner_offset':
                self.aruco_offset = float(p.value)
                self.get_logger().info(f'aruco_inner_offset → {self.aruco_offset}')
                grid_changed = True
            elif p.name == 'marker_sq_size':
                self.marker_sq = float(p.value)
                self.get_logger().info(f'marker_sq_size → {self.marker_sq}')
                grid_changed = True
            elif p.name == 'grid_dx':
                self.grid_dx = int(p.value)
                self.get_logger().info(f'grid_dx → {self.grid_dx}')
                grid_changed = True
            elif p.name == 'grid_dy':
                self.grid_dy = int(p.value)
                self.get_logger().info(f'grid_dy → {self.grid_dy}')
                grid_changed = True
        if grid_changed:
            # Rebuild H + grid_centres immediately from saved ArUco corners so the
            # grid overlay and nudge sliders respond live without waiting for idle.
            self._recompute_aruco_homography()
        return SetParametersResult(successful=True)

    def _cmd_cb(self, msg):
        cmd = msg.data.strip()
        if cmd == 'RECAL':
            if self.grid_centres:
                self._recompute_aruco_homography()
                self._ref_acc   = {}
                self._ref_count = 0
                self.empty_ref  = {}
                if self.state == self.WAIT_EMPTY:
                    # Initial setup: board just found, no pieces to remove → short settle,
                    # go to WAIT_PIECES afterward so user can place all 32 pieces
                    self._ref_delay  = 10
                    self._recal_mode = False
                    self.get_logger().info('RECAL (initial) — capturing empty board reference')
                else:
                    # Mid-game recal: arm removes pieces first (async Popen), use longer
                    # settle delay, then skip WAIT_PIECES and go straight back to TRACKING
                    self._ref_delay  = 60   # ~5s at 12Hz — wait for Gazebo teleports to complete
                    self._recal_mode = True
                    self.get_logger().info('RECAL (mid-game) — capturing empty board reference, will return to TRACKING')
                self.state = self.CAPTURING_REF
            else:
                self.get_logger().warn('RECAL ignored — no board grid yet')
        elif cmd == 'REMOVE_PIECES':
            if self.state in (self.WAIT_EMPTY, self.TRACKING, self.SEARCHING):
                self.state       = self.CAPTURING_REF
                self._ref_acc    = {}
                self._ref_count  = 0
                self._ref_delay  = 20  # skip ~1.5s of frames for pieces to settle
                self.get_logger().info(
                    'REMOVE_PIECES received — entering CAPTURING_REF (settling...)')
        elif cmd == 'RESET':
            # Game reset: keep the board grid and empty_ref so detection keeps working.
            # Only clear piece-tracking state so the game starts fresh.
            self.piece_sqs     = set()
            self.changed_sqs   = set()
            self.board_history = []
            self._awaiting_human_move = False
            if self.grid_centres and self.empty_ref:
                self.state = self.TRACKING
                # Pieces are teleported back via Popen (no arm motion → no _arm_just_idled).
                # Use a settle counter: after ~5s allow pieces to land, then auto-snapshot.
                self._reset_settle        = 60   # ~5s at 12Hz
                self._human_ref_sqs       = set()
                self._human_ref_gray      = {}
                self._prev_occ_set        = None
                self._move_stable_counter = 0
                self.get_logger().info('RESET — game reset, staying in TRACKING (settle 60 frames)')
            elif self.grid_centres:
                self.state = self.WAIT_EMPTY
                self.get_logger().info('RESET — game reset, back to WAIT_EMPTY (no empty ref)')
            else:
                self.state             = self.SEARCHING
                self._last_det_method  = ''
                self.get_logger().info('RESET — full restart (no grid found yet)')

    def _image_cb(self, msg):
        if self.camera_info is None:
            self._log_counter += 1
            if self._log_counter % self._log_interval == 1:
                self.get_logger().warn('Waiting for /camera/camera_info ...')
            return

        img  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Track marker stability every frame — cheap, only needs raw ArUco corner positions
        self._update_marker_stability(gray)

        self._overlay_edges(img, gray)

        if self.state == self.SEARCHING:
            self._run_searching(img, gray)

        elif self.state == self.WAIT_EMPTY:
            self._draw_board_outline(img)
            self._draw_tile_grid(img)
            self._draw_square_markers(img)

        elif self.state == self.CAPTURING_REF:
            self._draw_board_outline(img)
            self._draw_tile_grid(img)
            if self.arm_idle and self._markers_stable:
                self._accumulate_ref(gray, img)
            else:
                h, w = img.shape[:2]
                if not self.arm_idle:
                    wait_msg = 'Waiting for arm to stop...'
                else:
                    wait_msg = f'Waiting for camera to stabilise  ({self._marker_stable_ct}/{MARKER_STABLE_FRAMES})'
                cv2.putText(img, wait_msg, (8, h - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0),       4)
                cv2.putText(img, wait_msg, (8, h - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 100, 255), 1)

        elif self.state == self.WAIT_PIECES:
            self._run_wait_pieces(img, gray)
            self._draw_diff_overlay(img, gray)          # always on in tuning state

        elif self.state == self.TRACKING:
            self._run_tracking(img, gray)
            if self.debug_diff:
                self._draw_diff_overlay(img, gray)      # opt-in during tracking

        self._draw_hud(img)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))

    # ── State runners ─────────────────────────────────────────────────────────

    def _run_searching(self, img, gray):
        self._log_counter += 1

        if not self.arm_idle or not self._markers_stable:
            if not self.arm_idle:
                msg = 'Waiting for arm IDLE...'
            else:
                msg = f'Waiting for camera to stabilise  ({self._marker_stable_ct}/{MARKER_STABLE_FRAMES})'
            cv2.putText(img, msg, (8, img.shape[0] - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0),       4)
            cv2.putText(img, msg, (8, img.shape[0] - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 100, 255), 1)
            return

        # Both arm idle and camera stationary — try board detection
        found, method = self._run_all_board_detectors(gray, img)
        if found:
            self._last_det_method = method
            self.get_logger().info(
                f'Board found via {method} — {len(self.grid_centres)} in-frame squares')
            self.state = self.WAIT_EMPTY
        else:
            if self._log_counter % self._log_interval == 0:
                self.get_logger().info(
                    f'[SEARCHING] frame={self._log_counter} — no board detected')

    def _run_tracking(self, img, gray):
        self._log_counter += 1

        # All heavy vision work is gated on arm-just-idled AND marker stability.
        # _arm_just_idled fires on MOVING→IDLE but the arm may still be settling.
        # We wait for the camera to confirm it is truly stationary (markers stable),
        # but fall back after STABILITY_TIMEOUT frames so the game never gets stuck
        # if Gazebo controller oscillation prevents perfect stability.
        _STABILITY_TIMEOUT = 30   # frames at ~12 Hz
        if self._arm_just_idled:
            self._idle_stability_wait += 1
            _forced = self._idle_stability_wait >= _STABILITY_TIMEOUT
            if not self._markers_stable and self._idle_stability_wait % 10 == 0:
                self.get_logger().debug(
                    f'[TRACKING] Waiting for camera stability: '
                    f'{self._idle_stability_wait}/{_STABILITY_TIMEOUT}  '
                    f'stable={self._markers_stable}')

            if self._markers_stable or _forced:
                # Camera is stationary (or timeout reached) — take idle snapshot
                if _forced and not self._markers_stable:
                    self.get_logger().warn(
                        f'[TRACKING] Camera never stabilised — taking idle snapshot anyway '
                        f'(waited {self._idle_stability_wait} frames)')
                self._arm_just_idled = False
                self._idle_stability_wait = 0

                # Refresh board grid with all available detectors
                found, method = self._run_all_board_detectors(gray, img)
                if found:
                    self._last_det_method = method

                if self.empty_ref:
                    # Piece detection: diff vs empty-board reference
                    self.piece_sqs = set()
                    for sq in self.grid_centres:
                        cur   = self._sample_perspective(gray, sq)
                        empty = self.empty_ref.get(sq)
                        if empty is not None and abs(cur - empty) > self.piece_thr:
                            self.piece_sqs.add(sq)

                    # Change detection: diff vs previous idle snapshot
                    self.changed_sqs = set()
                    for sq in self.grid_centres:
                        prev = self.prev_idle_gray.get(sq)
                        cur  = self._sample_perspective(gray, sq)
                        if prev is not None and abs(cur - prev) > self.change_thr:
                            self.changed_sqs.add(sq)

                    # Store idle snapshot for next diff
                    self.prev_idle_gray = {
                        sq: self._sample_perspective(gray, sq)
                        for sq in self.grid_centres
                    }

                    # Publish per-square world XY positions (tile centres for now)
                    self._publish_piece_centroids()

                    # Record game history entry
                    self.board_history.append({
                        'frame':   self._log_counter,
                        'move':    self.last_move or '',
                        'pieces':  set(self.piece_sqs),
                        'changed': set(self.changed_sqs),
                    })

                    self.get_logger().info(
                        f'[TRACKING] idle snap  grid={len(self.grid_centres)}  '
                        f'pieces={len(self.piece_sqs)}  '
                        f'changed={sorted(self.changed_sqs)}  '
                        f'det={self._last_det_method}  '
                        f'history={len(self.board_history)}')

                    # Start monitoring for the human's response
                    self._human_ref_sqs       = set(self.piece_sqs)
                    self._human_ref_gray      = dict(self.prev_idle_gray)
                    self._prev_occ_set        = None
                    self._move_stable_counter = 0
                    self._awaiting_human_move = True

        # Post-RESET settle: pieces teleport back via Popen (no arm motion), so
        # _arm_just_idled never fires.  Count down AND require marker stability,
        # so we don't snapshot until Gazebo has finished all teleports.
        if self._reset_settle > 0 and self.empty_ref:
            self._reset_settle -= 1
            if self._reset_settle % 10 == 0:
                self.get_logger().debug(
                    f'[TRACKING] Post-RESET settle: {self._reset_settle} frames remaining  '
                    f'markers_stable={self._markers_stable}')
            if self._reset_settle == 0 and not self._markers_stable:
                self.get_logger().info(
                    '[TRACKING] Post-RESET: markers not yet stable — extending settle by 10 frames')
                self._reset_settle = 10   # markers still moving — extend wait
            if self._reset_settle == 0:
                self._human_ref_sqs = {
                    sq for sq in self.grid_centres
                    if (self.empty_ref.get(sq) is not None and
                        abs(self._sample_perspective(gray, sq) - self.empty_ref[sq]) > self.piece_thr)
                }
                self._human_ref_gray      = {sq: self._sample_perspective(gray, sq)
                                             for sq in self.grid_centres}
                self.piece_sqs            = set(self._human_ref_sqs)
                self._prev_occ_set        = None
                self._move_stable_counter = 0
                self._awaiting_human_move = True
                self.get_logger().info(
                    f'[TRACKING] Post-RESET snapshot: {len(self._human_ref_sqs)} pieces '
                    f'— awaiting human move')

        # Per-frame human move monitoring — require BOTH arm idle AND camera stationary
        if self.arm_idle and self._markers_stable and self._awaiting_human_move and self.empty_ref:
            self._monitor_human_move(gray)

        self._draw_board_outline(img)
        self._draw_tile_grid(img)
        self._draw_square_markers(img)

        out = String()
        out.data = json.dumps(sorted(self.piece_sqs))
        self.squares_pub.publish(out)

    # ── Board detection ───────────────────────────────────────────────────────

    def _run_all_board_detectors(self, gray, img=None):
        """Run board detectors. Returns (found: bool, method: str).

        ArUco is now the sole primary detector — it uses all 16 corners of all 4
        markers (LMEDS overdetermined homography) which gives accurate, perspective-
        correct grid alignment without any Hough assistance.

        Hough is commented out below rather than removed so it can be re-enabled
        as a fallback if the markers are partially occluded in a real-world setup.
        With a reliable 4-marker ArUco ring, running Hough on every frame adds CPU
        overhead and produces confusing cyan/yellow line clutter on the debug image
        without improving accuracy.
        """
        # Run ArUco — sets self.grid_centres and self.homography if successful
        aruco_ok = self._detect_via_aruco(gray, img)

        if aruco_ok:
            return True, 'ArUco'

        # ── Hough fallback (commented out — ArUco 16-pt LMEDS is the primary path) ──
        # Re-enable if ArUco markers are ever partially occluded:
        #
        # hough_ok = self._detect_via_hough(gray, img)
        # if hough_ok:
        #     return True, 'Hough'
        #
        # if self._log_counter % self._log_interval == 0:
        #     self.get_logger().info(
        #         f'Detectors: ArUco=no  Hough={"ok" if hough_ok else "no"}')
        # return hough_ok, 'Hough' if hough_ok else ''

        if self._log_counter % self._log_interval == 0:
            self.get_logger().info('Detectors: ArUco=no (all 4 markers needed)')
        return False, ''

    def _detect_via_aruco(self, gray, img=None) -> bool:
        """Detect the board using 4 ArUco markers placed at the board corners.

        Uses ALL 4 corners of ALL 4 markers (16 point correspondences) for
        findHomography so the system is highly overconstrained.  With LMEDS this
        averages out sub-pixel corner-detection errors and makes the homography
        robust to slight marker placement imperfections.

        Board-space corner positions are computed analytically from:
          • aruco_inner_offset  (d)    — gap from board edge to inner corner (squares)
          • marker_sq_size      (S)    — marker side length (0.060m / 0.045m = 4/3 sq)

        ArUco corner ordering TL→TR→BR→BL (clockwise in image), with camera looking
        down such that image_x ≈ board_y (file) and image_y ≈ board_x (rank):
          c0 TL: (bx_max, by_min)   c1 TR: (bx_max, by_max)
          c2 BR: (bx_min, by_max)   c3 BL: (bx_min, by_min)
        Per marker the bx/by bounds are:
          ID 0 a1: bx∈[-d-S,-d]  by∈[-d-S,-d]   → inner=c1 (TR) ✓ MARKER_INNER[0]=1
          ID 1 h1: bx∈[-d-S,-d]  by∈[8+d,8+d+S] → inner=c0 (TL) ✓ MARKER_INNER[1]=0
          ID 2 a8: bx∈[8+d,8+d+S] by∈[-d-S,-d]  → inner=c2 (BR) ✓ MARKER_INNER[2]=2
          ID 3 h8: bx∈[8+d,8+d+S] by∈[8+d,8+d+S]→ inner=c3 (BL) ✓ MARKER_INNER[3]=3
        Verified against chess_world.sdf geometry (see MARKER_SQ / _D constants).
        """
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

        if ids is None:
            return False

        id_map = {int(ids[i]): corners[i][0] for i in range(len(ids))}

        if img is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)

        if not {0, 1, 2, 3}.issubset(id_map):
            found = sorted(id_map.keys())
            if self._log_counter % self._log_interval == 0:
                self.get_logger().info(f'ArUco: found IDs {found}, need 0 1 2 3')
            return False

        # Save all 4 corners of every marker for live homography recompute
        self._last_aruco_corners  = {mid: id_map[mid].copy() for mid in range(4)}
        self._last_aruco_frame_hw = gray.shape[:2]

        # Build 16-point correspondence using all corners of all 4 markers
        d = self.aruco_offset
        S = self.marker_sq
        bx_min = {0: -d - S, 1: -d - S, 2: 8 + d,     3: 8 + d    }
        bx_max = {0: -d,     1: -d,     2: 8 + d + S,  3: 8 + d + S}
        by_min = {0: -d - S, 1: 8 + d,  2: -d - S,     3: 8 + d    }
        by_max = {0: -d,     1: 8 + d + S, 2: -d,      3: 8 + d + S}

        # Per-marker corner board-space positions (in squares × BOARD_SCALE):
        #   c0 (TL): (bx_max, by_min)  c1 (TR): (bx_max, by_max)
        #   c2 (BR): (bx_min, by_max)  c3 (BL): (bx_min, by_min)
        def corner_bpt(mid, ci):
            tbl = [(bx_max[mid], by_min[mid]),
                   (bx_max[mid], by_max[mid]),
                   (bx_min[mid], by_max[mid]),
                   (bx_min[mid], by_min[mid])]
            bx, by = tbl[ci]
            return [bx * BOARD_SCALE, by * BOARD_SCALE]

        src_all = []  # image-space pixels
        dst_all = []  # board-space (virtual px)
        for mid in range(4):
            for ci in range(4):
                px = id_map[mid][ci]
                src_all.append([float(px[0]), float(px[1])])
                dst_all.append(corner_bpt(mid, ci))

        src_all = np.float32(src_all)
        dst_all = np.float32(dst_all)

        # Overdetermined system (16 pts) — LMEDS averages out small corner errors
        H, _ = cv2.findHomography(dst_all, src_all, cv2.LMEDS)
        if H is None:
            return False

        # Project all 64 square centres through H, then apply pixel nudge.
        # bx = rank direction (bx=0 → rank8 in image, bx=8*BS → rank1 in image)
        # by = file direction (by=0 → h-file in image, by=8*BS → a-file in image)
        # Square name: FILES[7-fi] + str(8-ri)  (corrected for camera orientation)
        iw, ih = gray.shape[1], gray.shape[0]
        centres = {}
        for ri in range(8):
            for fi in range(8):
                bx = (ri + 0.5) * BOARD_SCALE   # rank index → bx (rank direction in world/H)
                by = (fi + 0.5) * BOARD_SCALE   # file index → by (file direction in world/H)
                pt = np.array([[[bx, by]]], dtype=np.float32)
                px = cv2.perspectiveTransform(pt, H)[0][0]
                u = int(round(float(px[0]))) + self.grid_dx
                v = int(round(float(px[1]))) + self.grid_dy
                if 0 <= u < iw and 0 <= v < ih:
                    centres[FILES[7 - fi] + str(8 - ri)] = (u, v)

        self.get_logger().info(
            f'ArUco: all 4 markers detected (16-pt LMEDS H)  in_frame={len(centres)}')

        if len(centres) < 4:
            return False

        self.grid_centres = centres
        self.homography   = H
        return True

    def _recompute_aruco_homography(self):
        """Recompute H and grid_centres from last saved ArUco corners + current params.

        Called immediately when aruco_inner_offset, marker_sq_size, grid_dx, or grid_dy
        change via the calibration GUI so the grid overlay updates live without waiting
        for a new frame or arm idle.  Uses the same 16-point LMEDS computation as
        _detect_via_aruco.  Does nothing if ArUco has never successfully detected markers.
        """
        if self._last_aruco_corners is None or self._last_aruco_frame_hw is None:
            return

        d = self.aruco_offset
        S = self.marker_sq
        bx_min = {0: -d - S, 1: -d - S, 2: 8 + d,     3: 8 + d    }
        bx_max = {0: -d,     1: -d,     2: 8 + d + S,  3: 8 + d + S}
        by_min = {0: -d - S, 1: 8 + d,  2: -d - S,     3: 8 + d    }
        by_max = {0: -d,     1: 8 + d + S, 2: -d,      3: 8 + d + S}

        def corner_bpt(mid, ci):
            tbl = [(bx_max[mid], by_min[mid]),
                   (bx_max[mid], by_max[mid]),
                   (bx_min[mid], by_max[mid]),
                   (bx_min[mid], by_min[mid])]
            bx, by = tbl[ci]
            return [bx * BOARD_SCALE, by * BOARD_SCALE]

        src_all, dst_all = [], []
        for mid in range(4):
            for ci in range(4):
                px = self._last_aruco_corners[mid][ci]
                src_all.append([float(px[0]), float(px[1])])
                dst_all.append(corner_bpt(mid, ci))

        H, _ = cv2.findHomography(np.float32(dst_all), np.float32(src_all), cv2.LMEDS)
        if H is None:
            return

        ih, iw = self._last_aruco_frame_hw
        centres = {}
        for ri in range(8):
            for fi in range(8):
                bx = (ri + 0.5) * BOARD_SCALE   # rank → bx
                by = (fi + 0.5) * BOARD_SCALE   # file → by
                pt = np.array([[[bx, by]]], dtype=np.float32)
                px = cv2.perspectiveTransform(pt, H)[0][0]
                u = int(round(float(px[0]))) + self.grid_dx
                v = int(round(float(px[1]))) + self.grid_dy
                if 0 <= u < iw and 0 <= v < ih:
                    centres[FILES[7 - fi] + str(8 - ri)] = (u, v)
        if len(centres) >= 4:
            self.homography   = H
            self.grid_centres = centres

    def _detect_via_hough(self, gray, img=None) -> bool:
        """Detect the board grid from straight tile edges using Hough line transform.

        Works on geometry rather than colour — robust to pieces obscuring tiles.
        Populates self.grid_centres with up to 64 square centres if ≥4 are in frame.
        """
        h, w = gray.shape[:2]

        edges = cv2.Canny(gray, 50, 150)
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=40,
                                minLineLength=30, maxLineGap=15)
        if lines is None:
            return False

        # ── Perspective-aware line classification via angle clustering ───────────
        # Instead of assuming near-H / near-V, find the two dominant grid directions
        # from the data — works for any camera angle including significant tilt.

        all_segs = []
        seg_angles  = []
        seg_lengths = []
        for x1, y1, x2, y2 in lines[:, 0]:
            L = float(np.hypot(x2 - x1, y2 - y1))
            if L < 1:
                continue
            a = float(np.arctan2(y2 - y1, x2 - x1)) % np.pi   # fold to [0, π)
            all_segs.append((x1, y1, x2, y2))
            seg_angles.append(a)
            seg_lengths.append(L)

        if len(all_segs) < 4:
            return False

        seg_angles  = np.array(seg_angles)
        seg_lengths = np.array(seg_lengths)

        # Length-weighted angle histogram (36 bins × 5°)
        N_BINS = 36
        hist, edges = np.histogram(seg_angles, bins=N_BINS,
                                   range=(0, np.pi), weights=seg_lengths)
        bin_centres = (edges[:-1] + edges[1:]) / 2

        # Dominant direction A
        p1      = int(np.argmax(hist))
        angle_A = bin_centres[p1]

        # Dominant direction B — closest bin to A+90° with ≥10% of peak weight
        target      = (angle_A + np.pi / 2) % np.pi
        ang_dist_to_target = np.abs(bin_centres - target)
        ang_dist_to_target = np.minimum(ang_dist_to_target, np.pi - ang_dist_to_target)
        candidates  = np.where(hist >= 0.10 * hist[p1])[0]
        if len(candidates) == 0:
            return False
        p2      = int(candidates[np.argmin(ang_dist_to_target[candidates])])
        angle_B = bin_centres[p2]

        def _ang_dist(a, b):
            d = abs(a - b) % np.pi
            return min(d, np.pi - d)

        # Classify each segment into group A or B (tolerance ±25.7°)
        TOL = np.pi / 7
        groupA, groupB = [], []
        for seg, a in zip(all_segs, seg_angles):
            dA = _ang_dist(a, angle_A)
            dB = _ang_dist(a, angle_B)
            if dA < TOL and dA <= dB:
                groupA.append(seg)
            elif dB < TOL and dB < dA:
                groupB.append(seg)

        # Assign H / V: group whose dominant angle is closer to 0 (horizontal) is H
        h_segs = groupA if _ang_dist(angle_A, 0) <= _ang_dist(angle_B, 0) else groupB
        v_segs = groupB if h_segs is groupA else groupA
        h_angle = angle_A if h_segs is groupA else angle_B
        v_angle = angle_B if h_segs is groupA else angle_A

        def cluster_by_perp(segs, dominant_angle):
            """Cluster segments by their midpoint projected onto the perpendicular axis.
            Works for any line direction — handles perspective-converging lines."""
            if not segs:
                return []
            perp = dominant_angle + np.pi / 2
            nx, ny = np.cos(perp), np.sin(perp)
            positions = sorted(
                (s[0] + s[2]) / 2 * nx + (s[1] + s[3]) / 2 * ny
                for s in segs
            )
            clusters, group = [], [positions[0]]
            for pos in positions[1:]:
                if pos - group[-1] <= 15:
                    group.append(pos)
                else:
                    clusters.append(float(np.median(group)))
                    group = [pos]
            clusters.append(float(np.median(group)))
            return clusters

        h_clusters = cluster_by_perp(h_segs, h_angle)
        v_clusters = cluster_by_perp(v_segs, v_angle)

        n_h, n_v = len(h_clusters), len(v_clusters)
        if n_h < 2 or n_v < 2:
            if self._log_counter % self._log_interval == 0:
                self.get_logger().info(
                    f'Hough: H={n_h} V={n_v} (angles A={np.degrees(angle_A):.0f}° '
                    f'B={np.degrees(angle_B):.0f}°) — not enough clusters')
            return False

        # Debug: draw detected lines
        if img is not None:
            for x1, y1, x2, y2 in h_segs:
                cv2.line(img, (x1, y1), (x2, y2), (255, 255, 0), 1)   # cyan
            for x1, y1, x2, y2 in v_segs:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 255), 1)   # yellow

        # Compute pairwise intersections
        intersections = [(int(vx), int(hy))
                         for hy in h_clusters for vx in v_clusters]
        if img is not None:
            for pt in intersections:
                cv2.circle(img, pt, 4, (255, 255, 255), -1)

        # Estimate grid step from cluster spacing
        def median_step(positions):
            s = sorted(positions)
            diffs = [s[i + 1] - s[i] for i in range(len(s) - 1)]
            return float(np.median(diffs)) if diffs else 0.0

        step_h = median_step(h_clusters)
        step_v = median_step(v_clusters)
        if step_h <= 0 or step_v <= 0:
            return False

        # Extrapolate to cover all 9 grid lines (8 squares + 1)
        def extrapolate_to_9(positions, step):
            s = sorted(positions)
            while len(s) < 9:
                # Extend whichever end needs more lines
                if 9 - len(s) > 0:
                    s.insert(0, s[0] - step)
                if len(s) < 9:
                    s.append(s[-1] + step)
            return s[:9]

        h9 = extrapolate_to_9(h_clusters, step_h)  # 9 horizontal y-positions
        v9 = extrapolate_to_9(v_clusters, step_v)  # 9 vertical   x-positions

        # Derive 8×8 square centres as midpoints of 2×2 corner quads
        # Image y increases downward → h9[0] is top of image → rank 8
        centres = {}
        for ri in range(8):
            rank = 8 - ri          # top row → rank 8
            for fi in range(8):
                sq_name = FILES[fi] + str(rank)
                cu = (v9[fi] + v9[fi + 1]) / 2
                cv_coord = (h9[ri] + h9[ri + 1]) / 2
                u, v = int(round(cu)), int(round(cv_coord))
                if 0 <= u < w and 0 <= v < h:
                    centres[sq_name] = (u, v)

        n_int = len(intersections)
        n_in  = len(centres)
        self.get_logger().info(
            f'Hough: H={n_h} V={n_v} intersections={n_int} in_frame={n_in}')

        if n_in < 4:
            return False

        self.grid_centres = centres
        return True

    # ── Reference capture + sampling ─────────────────────────────────────────

    def _accumulate_ref(self, gray, img):
        """Collect frames into empty-board reference average."""
        if self._ref_delay > 0:
            self._ref_delay -= 1
            h, w = img.shape[:2]
            cv2.putText(img, f'Settling... {self._ref_delay} frames',
                        (8, img.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 0, 0),       4)
            cv2.putText(img, f'Settling... {self._ref_delay} frames',
                        (8, img.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (100, 200, 255), 1)
            return

        for sq in self.grid_centres:
            self._ref_acc.setdefault(sq, []).append(self._sample_perspective(gray, sq))
        self._ref_count += 1

        # Progress bar
        h, w = img.shape[:2]
        pct  = self._ref_count / self.REF_FRAMES_NEEDED
        bar_w = int(w * pct)
        cv2.rectangle(img, (0, h - 8), (w, h),       (0, 0, 0),     -1)
        cv2.rectangle(img, (0, h - 8), (bar_w, h),   (0, 200, 80),  -1)
        cv2.putText(img,
                    f'Capturing empty board ref  {self._ref_count}/{self.REF_FRAMES_NEEDED}',
                    (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1)

        if self._ref_count >= self.REF_FRAMES_NEEDED:
            self.empty_ref      = {sq: float(np.mean(v))
                                   for sq, v in self._ref_acc.items()}
            self.prev_idle_gray = dict(self.empty_ref)
            self.piece_sqs      = set()
            self.changed_sqs    = set()
            self.board_history  = []
            if self._recal_mode:
                # Mid-game recal: pieces will be returned to board, go straight to TRACKING
                self._recal_mode = False
                self.state = self.TRACKING
                self.get_logger().info(
                    f'Empty board reference captured ({len(self.empty_ref)} squares) '
                    f'— returning to TRACKING (mid-game recal)')
            else:
                # Initial setup: wait for all 32 pieces to be placed
                self.state = self.WAIT_PIECES
                self.get_logger().info(
                    f'Empty board reference captured ({len(self.empty_ref)} squares) '
                    f'— waiting for 32 pieces to be placed (WAIT_PIECES)')

    def _run_wait_pieces(self, img, gray):
        """Wait until all 32 starting squares are occupied.

        Samples only when arm is idle AND camera is stationary (human placing pieces).
        Once all STARTING_SQUARES show brightness diff > piece_thr vs empty_ref,
        take a prev_idle_gray snapshot and enter TRACKING.
        """
        self._draw_board_outline(img)
        self._draw_tile_grid(img)

        # Only sample when arm is idle AND camera is stationary — reuse last result otherwise
        if self.arm_idle and self._markers_stable:
            occupied = set()
            missing  = set()
            for sq in STARTING_SQUARES:
                if self.grid_centres.get(sq) is None:
                    missing.add(sq)
                    continue
                cur   = self._sample_perspective(gray, sq)
                empty = self.empty_ref.get(sq)
                if empty is not None and abs(cur - empty) > self.piece_thr:
                    occupied.add(sq)
                else:
                    missing.add(sq)
            self._wp_occupied = occupied
            self._wp_missing  = missing
        else:
            occupied = self._wp_occupied
            missing  = self._wp_missing

        h, w = img.shape[:2]
        r = self.sample_r + 10

        # Draw occupied starting squares in green, missing in red
        for sq in STARTING_SQUARES:
            uv = self.grid_centres.get(sq)
            if uv is None:
                continue
            u, v = uv
            if not (0 <= u < w and 0 <= v < h):
                continue
            if sq in occupied:
                cv2.circle(img, (u, v), r, (0, 0, 0),    4)
                cv2.circle(img, (u, v), r, (0, 220, 60), 2)   # green — piece present
            else:
                cv2.circle(img, (u, v), r, (0, 0, 0),    4)
                cv2.circle(img, (u, v), r, (0, 60, 220), 2)   # red — piece missing

        # Progress bar
        n = len(occupied)
        pct   = n / 32
        bar_w = int(w * pct)
        cv2.rectangle(img, (0, h - 8), (w, h),       (0, 0, 0),    -1)
        cv2.rectangle(img, (0, h - 8), (bar_w, h),   (0, 180, 60), -1)
        cv2.putText(img, f'Place all pieces  {n}/32 detected',
                    (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        self._log_counter += 1
        if self._log_counter % self._log_interval == 0:
            self.get_logger().info(
                f'[WAIT_PIECES] {n}/32 occupied  missing={sorted(missing)[:8]}...')

        if n >= 32:
            # All pieces on board — snapshot prev_idle and start tracking
            self.prev_idle_gray = {
                sq: self._sample_perspective(gray, sq)
                for sq in self.grid_centres
            }
            self.piece_sqs   = set(occupied)
            self.changed_sqs = set()
            self.board_history = []
            self.state = self.TRACKING
            # Human plays first — start monitoring immediately
            self._human_ref_sqs       = set(occupied)
            self._human_ref_gray      = dict(self.prev_idle_gray)
            self._prev_occ_set        = None
            self._move_stable_counter = 0
            self._awaiting_human_move = True
            self.get_logger().info('All 32 pieces detected — entering TRACKING (awaiting human move)')

    def _update_marker_stability(self, gray):
        """Detect ArUco markers and track centroid drift to determine if arm is truly still.

        Runs every frame once a homography exists.  Cheaper than a full homography
        recompute — only needs the raw corner positions.  Updates:
          _markers_stable   — True when centroid has drifted < MARKER_STABLE_THR px
                              for MARKER_STABLE_FRAMES consecutive frames.
          _marker_stable_ct — running counter of stable frames.
        """
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

        if ids is None or not {0, 1, 2, 3}.issubset({int(ids[i]) for i in range(len(ids))}):
            # Lost sight of markers — reset stability (arm may be occluding them)
            if self._markers_stable:
                found_ids = sorted(int(ids[i]) for i in range(len(ids))) if ids is not None else []
                self.get_logger().warn(
                    f'[CAM] Markers lost (found {found_ids}) — stability reset')
            self._marker_stable_ct = 0
            self._markers_stable   = False
            self._prev_marker_ctr  = None
            return

        id_map  = {int(ids[i]): corners[i][0] for i in range(len(ids))}
        all_pts = np.concatenate([id_map[mid] for mid in range(4)], axis=0)
        ctr     = np.mean(all_pts, axis=0)   # shape (2,)

        prev_stable = self._markers_stable
        if self._prev_marker_ctr is not None:
            drift = float(np.linalg.norm(ctr - self._prev_marker_ctr))
            if drift < MARKER_STABLE_THR:
                self._marker_stable_ct = min(self._marker_stable_ct + 1,
                                             MARKER_STABLE_FRAMES + 1)
            else:
                if self._marker_stable_ct > 0:
                    self.get_logger().debug(
                        f'[CAM] Drift {drift:.1f}px — stability counter reset '
                        f'(was {self._marker_stable_ct})')
                self._marker_stable_ct = 0

        self._prev_marker_ctr = ctr
        self._markers_stable  = self._marker_stable_ct >= MARKER_STABLE_FRAMES

        # Log state transitions only
        if self._markers_stable and not prev_stable:
            self.get_logger().info(
                f'[CAM] Markers stable — camera stationary  ctr=({ctr[0]:.1f},{ctr[1]:.1f})')
        elif not self._markers_stable and prev_stable:
            self.get_logger().info('[CAM] Markers moving — camera no longer stationary')

    def _monitor_human_move(self, gray):
        """Detect a stable piece movement since the human reference snapshot.

        Called every frame while arm is idle and _awaiting_human_move is True.
        Compares current board occupancy to _human_ref_sqs.  Once the change is
        stable for MOVE_STABLE_FRAMES consecutive frames, classifies the move:

          Simple move: 1 square disappeared + 1 new square appeared → from + to
          Capture:     1 square disappeared + 0 new squares           → from + changed occupied

        Publishes UCI string to /chess/human_move and clears _awaiting_human_move.
        """
        # Build current occupancy map
        cur_occupied = set()
        for sq in self.grid_centres:
            cur  = self._sample_perspective(gray, sq)
            empty = self.empty_ref.get(sq)
            if empty is not None and abs(cur - empty) > self.piece_thr:
                cur_occupied.add(sq)

        # Stability tracking — reset counter on any change in occupied set
        if cur_occupied == self._prev_occ_set:
            self._move_stable_counter += 1
        else:
            if self._prev_occ_set is not None:
                changed = (cur_occupied ^ self._prev_occ_set)
                self.get_logger().debug(
                    f'[MONITOR] Occupancy changed — reset stability  '
                    f'delta={sorted(changed)}  occ={len(cur_occupied)}')
            self._prev_occ_set        = set(cur_occupied)
            self._move_stable_counter = 0
            return

        if self._move_stable_counter < MOVE_STABLE_FRAMES:
            return

        # Board is stable — check if it actually changed vs reference
        if cur_occupied == self._human_ref_sqs:
            return   # no change yet

        disappeared = self._human_ref_sqs - cur_occupied   # pieces that moved away
        appeared    = cur_occupied - self._human_ref_sqs   # pieces on previously empty squares

        self.get_logger().info(
            f'[MONITOR] Stable board change after {self._move_stable_counter} frames  '
            f'disappeared={sorted(disappeared)}  appeared={sorted(appeared)}')

        uci = None

        if len(disappeared) == 1 and len(appeared) == 1:
            # Simple move — piece lifted from one square, placed on another empty square
            uci = next(iter(disappeared)) + next(iter(appeared))

        elif len(disappeared) == 1 and len(appeared) == 0:
            # Possible capture — to_sq was already occupied; look for a brightness change
            # on a square that was in reference (still shows occupied but different piece)
            from_sq = next(iter(disappeared))
            changed_occupied = set()
            for sq in (self._human_ref_sqs - disappeared):
                cur  = self._sample_perspective(gray, sq)
                ref  = self._human_ref_gray.get(sq)
                if ref is not None and abs(cur - ref) > self.change_thr:
                    changed_occupied.add(sq)
            self.get_logger().info(
                f'[MONITOR] Capture candidate: from={from_sq}  '
                f'changed_occupied={sorted(changed_occupied)}')
            if len(changed_occupied) == 1:
                uci = from_sq + next(iter(changed_occupied))
            elif len(changed_occupied) == 0:
                self.get_logger().warn(
                    f'[MONITOR] Capture detection failed — no brightness change found '
                    f'on remaining occupied squares (change_thr={self.change_thr:.0f})')
            else:
                self.get_logger().warn(
                    f'[MONITOR] Ambiguous capture — {len(changed_occupied)} changed squares: '
                    f'{sorted(changed_occupied)}')
        else:
            self.get_logger().warn(
                f'[MONITOR] Unclassified board change — '
                f'disappeared={sorted(disappeared)} appeared={sorted(appeared)} '
                f'(expected 1+1 or 1+0)')

        if uci:
            self.get_logger().info(f'[MONITOR] Publishing human move: {uci}')
            msg = String()
            msg.data = uci
            self.human_move_pub.publish(msg)
            self._awaiting_human_move = False
            # Update piece_sqs to reflect new state immediately
            self.piece_sqs = set(cur_occupied)

    def _sample(self, gray, uv) -> float:
        u, v = uv
        h, w = gray.shape
        r    = self.sample_r
        patch = gray[max(0, v - r):min(h, v + r), max(0, u - r):min(w, u + r)]
        return float(np.mean(patch)) if patch.size > 0 else 0.0

    def _sample_perspective(self, gray, sq) -> float:
        """Sample a tile by warping its perspective quadrilateral to a canonical square.

        When the ArUco homography is available the tile's four projected corners
        define the source quad, which is warped to a 32×32 px square — exact
        regardless of camera angle.  The inner 60% is averaged to exclude tile-edge
        shadows and piece-base bleed into adjacent tiles.

        Falls back to square-patch _sample() when no homography is set.
        """
        fi = FILES.index(sq[0])
        ri = int(sq[1]) - 1          # rank 1 → index 0

        if self.homography is None:
            uv = self.grid_centres.get(sq)
            return self._sample(gray, uv) if uv else 0.0

        H             = self.homography
        SZ            = 32
        h_img, w_img  = gray.shape[:2]

        # Project the 4 corners of this tile through H, apply pixel nudge
        src = []
        for dfi, dri in [(0, 0), (1, 0), (1, 1), (0, 1)]:
            bx = (7 - ri + dri) * BOARD_SCALE   # flipped rank
            by = (7 - fi + dfi) * BOARD_SCALE   # flipped file
            pt = np.array([[[bx, by]]], dtype=np.float32)
            px = cv2.perspectiveTransform(pt, H)[0][0]
            src.append([float(px[0]) + self.grid_dx,
                        float(px[1]) + self.grid_dy])
        src = np.float32(src)

        # Bail if tile is entirely outside the frame
        if (np.all(src[:, 0] < 0) or np.all(src[:, 0] > w_img) or
                np.all(src[:, 1] < 0) or np.all(src[:, 1] > h_img)):
            uv = self.grid_centres.get(sq)
            return self._sample(gray, uv) if uv else 0.0

        dst = np.float32([[0, 0], [SZ, 0], [SZ, SZ], [0, SZ]])
        M   = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(gray, M, (SZ, SZ))

        # Inner 60% — avoids tile-edge shadows and piece-base overlap
        margin = int(SZ * 0.2)
        inner  = warped[margin:SZ - margin, margin:SZ - margin]
        return float(np.mean(inner)) if inner.size > 0 else 0.0

    # ── Piece centroid computation ─────────────────────────────────────────────

    def _compute_piece_centroids(self) -> dict:
        """Return world XY for each occupied square.

        Currently: tile centre (square_to_xyz equivalent).
        TODO: back-project camera brightness-weighted centroid for sub-tile accuracy.
        """
        result = {}
        for sq in self.piece_sqs:
            fi = FILES.index(sq[0])
            ri = int(sq[1]) - 1   # rank 1 → 0
            # Tile centre in world coords (mirrors chess_arm_node.square_to_xyz, no flip)
            world_x = self.ox + (ri + 0.5) * self.sq
            world_y = self.oy + (fi + 0.5) * self.sq
            result[sq] = (round(world_x, 4), round(world_y, 4))
        return result

    def _publish_piece_centroids(self):
        """Publish per-square world XY centroids for all occupied squares."""
        if not self.arm_idle or not self._markers_stable:
            return
        centroids = self._compute_piece_centroids()
        if not centroids:
            return
        msg = String()
        msg.data = json.dumps(centroids)
        self.centroids_pub.publish(msg)
        self.get_logger().info(
            f'[CENTROIDS] published {len(centroids)} tile-centre positions  '
            f'sq_list={sorted(centroids.keys())}')

    # ── Debug drawing layers ───────────────────────────────────────────────────

    def _draw_diff_overlay(self, img, gray):
        """Per-square brightness diff vs empty_ref shown as colour-coded numbers.

        Color scale (relative to piece_thr):
          diff < 0.5×thr  → dark gray   (clearly empty)
          0.5–1.0×thr     → yellow      (borderline — threshold may need adjusting)
          > 1.0×thr       → green       (clearly occupied)

        Always visible in WAIT_PIECES. In TRACKING requires debug_diff=True.

        Sampling is gated on arm_idle AND _markers_stable — the cache is frozen
        while the arm is moving or the camera is still settling, so the overlay
        always shows the last known stable values rather than noisy in-motion data.
        """
        if not self.empty_ref:
            return

        # Recompute only when arm is idle and camera is stationary
        if self.arm_idle and self._markers_stable:
            new_cache = {}
            for sq in self.grid_centres:
                ref = self.empty_ref.get(sq)
                if ref is None:
                    continue
                cur = self._sample_perspective(gray, sq)
                new_cache[sq] = abs(cur - ref)
            self._diff_cache = new_cache

        if not self._diff_cache:
            return

        h, w = img.shape[:2]
        thr = self.piece_thr
        for sq, uv in self.grid_centres.items():
            u, v = uv
            if not (0 <= u < w and 0 <= v < h):
                continue
            diff = self._diff_cache.get(sq)
            if diff is None:
                continue
            if diff < 0.5 * thr:
                color = (90, 90, 90)      # gray — empty
            elif diff < thr:
                color = (0, 200, 220)     # yellow — borderline
            else:
                color = (60, 220, 60)     # green — occupied
            cv2.putText(img, f'{diff:.0f}', (u - 10, v + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 3)
            cv2.putText(img, f'{diff:.0f}', (u - 10, v + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color,     1)

    def _overlay_edges(self, img, gray):
        """Tinted Canny edge overlay — shows board/tile boundaries."""
        edges = cv2.Canny(gray, 40, 120)
        edge_layer = np.zeros_like(img)
        edge_layer[edges > 0] = (0, 160, 0)    # green — doesn't contaminate markers
        cv2.addWeighted(img, 1.0, edge_layer, 0.5, 0, img)

    def _draw_board_outline(self, img):
        """Draw a bounding rectangle around all in-frame grid centres."""
        if not self.grid_centres:
            return
        h, w = img.shape[:2]
        pts = [(u, v) for u, v in self.grid_centres.values()
               if 0 <= u < w and 0 <= v < h]
        if not pts:
            return
        us, vs = zip(*pts)
        pad = self.sample_r + 8
        x1, y1 = max(0, min(us) - pad), max(0, min(vs) - pad)
        x2, y2 = min(w, max(us) + pad), min(h, max(vs) + pad)
        # Triple rectangle: black shadow, white highlight, green outline
        cv2.rectangle(img, (x1 - 4, y1 - 4), (x2 + 4, y2 + 4), (0,   0,   0), 8)
        cv2.rectangle(img, (x1 - 2, y1 - 2), (x2 + 2, y2 + 2), (255, 255, 255), 4)
        cv2.rectangle(img, (x1 - 2, y1 - 2), (x2 + 2, y2 + 2), (0,   255,   0), 2)
        cv2.putText(img, 'BOARD', (x1 + 4, max(16, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0),     5)
        cv2.putText(img, 'BOARD', (x1 + 4, max(16, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0),   2)

    def _draw_tile_grid(self, img):
        """Draw actual tile boundary lines on the debug image.

        When the ArUco homography is available: projects the full 9×9 corner grid
        through H so lines fall on real tile edges (not centre-to-centre).
        Falls back to centre-to-centre lines when only Hough grid is available.
        File labels (a–h) and rank labels (1–8) drawn at the board edges.
        """
        h, w = img.shape[:2]

        def in_frame(u, v):
            return 0 <= u < w and 0 <= v < h

        if self.homography is not None:
            # ── Homography path: project 9×9 corners → draw tile edges ──────────
            H = self.homography
            corners = {}
            for ri in range(9):
                for fi in range(9):
                    bx = ri * BOARD_SCALE   # rank → bx
                    by = fi * BOARD_SCALE   # file → by
                    pt = np.array([[[bx, by]]], dtype=np.float32)
                    px = cv2.perspectiveTransform(pt, H)[0][0]
                    u = int(round(float(px[0]))) + self.grid_dx
                    v = int(round(float(px[1]))) + self.grid_dy
                    corners[(fi, ri)] = (u, v)

            # Semi-transparent checkerboard overlay — makes misalignment immediately
            # visible: if a shaded quad covers the wrong physical tile colour, the
            # aruco_inner_offset / grid_dx / grid_dy params need adjustment.
            overlay = img.copy()
            for ri in range(8):
                for fi in range(8):
                    quad = np.array([
                        corners[(fi,     ri)],
                        corners[(fi + 1, ri)],
                        corners[(fi + 1, ri + 1)],
                        corners[(fi,     ri + 1)],
                    ], dtype=np.int32)
                    color = (210, 210, 210) if (fi + ri) % 2 == 0 else (45, 45, 45)
                    cv2.fillPoly(overlay, [quad], color)
            cv2.addWeighted(overlay, 0.18, img, 0.82, 0, img)

            # Horizontal tile edges (constant ri)
            for ri in range(9):
                for fi in range(8):
                    p1, p2 = corners[(fi, ri)], corners[(fi + 1, ri)]
                    if in_frame(*p1) or in_frame(*p2):
                        cv2.line(img, p1, p2, (0,   0,   0), 3)
                        cv2.line(img, p1, p2, (0, 160,  60), 1)

            # Vertical tile edges (constant fi)
            for fi in range(9):
                for ri in range(8):
                    p1, p2 = corners[(fi, ri)], corners[(fi, ri + 1)]
                    if in_frame(*p1) or in_frame(*p2):
                        cv2.line(img, p1, p2, (0,   0,   0), 3)
                        cv2.line(img, p1, p2, (0, 160,  60), 1)

            # File labels (a–h) along bottom edge
            for fi in range(8):
                p = corners[(fi, 8)]
                if in_frame(*p):
                    cv2.putText(img, FILES[fi], (p[0] - 4, min(h - 4, p[1] + 14)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,   0,   0), 3)
                    cv2.putText(img, FILES[fi], (p[0] - 4, min(h - 4, p[1] + 14)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200,  80), 1)

            # Rank labels (1–8) along left edge (bx=0 = rank-1 side, ri increases → rank increases)
            for ri in range(8):
                p = corners[(0, ri)]
                rank = ri + 1
                if in_frame(*p):
                    cv2.putText(img, str(rank), (max(0, p[0] - 14), p[1] + 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,   0,   0), 3)
                    cv2.putText(img, str(rank), (max(0, p[0] - 14), p[1] + 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200,  80), 1)

        else:
            # ── Fallback: centre-to-centre lines when only Hough grid available ──
            for ri in range(8):
                for fi in range(8):
                    sq  = FILES[fi] + str(ri + 1)
                    uv  = self.grid_centres.get(sq)
                    if not uv or not in_frame(*uv):
                        continue
                    if fi < 7:
                        sq2 = FILES[fi + 1] + str(ri + 1)
                        uv2 = self.grid_centres.get(sq2)
                        if uv2 and in_frame(*uv2):
                            cv2.line(img, uv, uv2, (0,   0,   0), 3)
                            cv2.line(img, uv, uv2, (80, 80,  80), 1)
                    if ri < 7:
                        sq2 = FILES[fi] + str(ri + 2)
                        uv2 = self.grid_centres.get(sq2)
                        if uv2 and in_frame(*uv2):
                            cv2.line(img, uv, uv2, (0,   0,   0), 3)
                            cv2.line(img, uv, uv2, (80, 80,  80), 1)

    def _draw_square_markers(self, img):
        """Draw per-square indicators: pieces, changed squares, empty squares."""
        h, w = img.shape[:2]
        r = self.sample_r + 14

        for sq, (u, v) in self.grid_centres.items():
            if not (0 <= u < w and 0 <= v < h):
                continue

            is_piece   = sq in self.piece_sqs
            is_changed = sq in self.changed_sqs

            if is_changed:
                # Magenta — square changed since last idle frame
                cv2.circle(img, (u, v), r + 4, (0,   0,   0),   8)
                cv2.circle(img, (u, v), r + 4, (255,  0, 255),  5)
                cv2.putText(img, sq, (u - 14, v - r - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,   0,   0),   6)
                cv2.putText(img, sq, (u - 14, v - r - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,  0, 255),  2)

            if is_piece:
                # Orange ring + crosshair — piece present (diff vs empty ref)
                cv2.circle(img, (u, v), r, (0,   0,   0),   6)
                cv2.circle(img, (u, v), r, (0, 165, 255),   4)
                cv2.line(img, (u - r, v),     (u + r, v),     (0,   0,   0), 4)
                cv2.line(img, (u - r, v),     (u + r, v),     (0, 165, 255), 2)
                cv2.line(img, (u,     v - r), (u,     v + r), (0,   0,   0), 4)
                cv2.line(img, (u,     v - r), (u,     v + r), (0, 165, 255), 2)
                cv2.putText(img, sq, (u - 14, v - r - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,   0,   0),   6)
                cv2.putText(img, sq, (u - 14, v - r - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255),   2)
            elif not is_changed:
                # Small gray dot — empty square
                cv2.circle(img, (u, v), 5, (0,   0,   0),   -1)
                cv2.circle(img, (u, v), 4, (140, 140, 140), -1)

    def _draw_hud(self, img):
        """Status bar + state-specific instructions."""
        h, w = img.shape[:2]
        state_color = {
            self.SEARCHING:     (0,  80, 255),   # red-orange
            self.WAIT_EMPTY:    (0, 200, 255),   # yellow
            self.CAPTURING_REF: (200, 200, 0),   # cyan
            self.WAIT_PIECES:   (60, 200, 255),  # orange — waiting for pieces
            self.TRACKING:      (0, 220,   0),   # green
        }.get(self.state, (200, 200, 200))

        cv2.rectangle(img, (0, 0), (w, 54), (0, 0, 0), -1)

        extra = ''
        if self.state == self.TRACKING:
            extra = (f'  pieces={len(self.piece_sqs)}  changed={len(self.changed_sqs)}'
                     f'  snap={len(self.board_history)}')
        elif self.state == self.CAPTURING_REF:
            extra = f'  {self._ref_count}/{self.REF_FRAMES_NEEDED} frames'
        elif self.state == self.WAIT_PIECES:
            n = sum(1 for sq in STARTING_SQUARES if sq in self.piece_sqs)
            extra = f'  {len(self.piece_sqs)}/32 pieces'

        det = f'  det={self._last_det_method}' if self._last_det_method else ''
        arm = '  ARM:IDLE' if self.arm_idle else '  ARM:MOVING'
        stable = '  CAM:STILL' if self._markers_stable else f'  CAM:DRIFT({self._marker_stable_ct}/{MARKER_STABLE_FRAMES})'
        thr = ''
        if self.state in (self.WAIT_PIECES, self.TRACKING):
            thr = f'  thr={self.piece_thr:.0f}/{self.change_thr:.0f}'
        label = f'{self.state}   grid={len(self.grid_centres)}{extra}{det}{thr}{arm}{stable}'
        cv2.putText(img, label, (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0),   6)
        cv2.putText(img, label, (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.75, state_color, 2)

        # Centre instruction overlay
        cx, cy = w // 2, h // 2
        if self.state == self.SEARCHING:
            if not self.arm_idle:
                label2 = 'WAITING FOR ARM IDLE'
                cv2.putText(img, label2, (cx - 190, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,   0,   0),   8)
                cv2.putText(img, label2, (cx - 190, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.1, (100, 100, 255), 3)
            elif not self._markers_stable:
                label2 = f'CAMERA SETTLING  {self._marker_stable_ct}/{MARKER_STABLE_FRAMES}'
                cv2.putText(img, label2, (cx - 200, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,   0,   0),   8)
                cv2.putText(img, label2, (cx - 200, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 200, 255), 3)
            else:
                cv2.putText(img, 'BOARD NOT FOUND', (cx - 160, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,  0, 0), 8)
                cv2.putText(img, 'BOARD NOT FOUND', (cx - 160, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 80, 255), 3)
        elif self.state == self.WAIT_EMPTY:
            msg = 'Board locked — click  Calibrate Camera  then wait'
            cv2.putText(img, msg, (cx - 200, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,   0,   0), 6)
            cv2.putText(img, msg, (cx - 200, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
        elif self.state == self.WAIT_PIECES:
            msg = 'Place all 32 pieces on starting squares'
            cv2.putText(img, msg, (cx - 220, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,   0,   0), 6)
            cv2.putText(img, msg, (cx - 220, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (60, 200, 255), 2)

def main(args=None):
    rclpy.init(args=args)
    node = ChessVisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
