#!/usr/bin/env python3
"""
chess_vision_node.py — Board-first piece tracking.

Pipeline:
  SEARCHING  → scan every frame for the chess board using findChessboardCorners.
               Falls back to tile-color detection if corners not found.
  BOARD_FOUND→ board located, grid centres computed, waiting for arm IDLE
               to capture reference frame.
  TRACKING   → frame-diff at each grid centre to detect piece moves.

Publishes:
  /chess/vision/white_squares  — JSON list of squares with white pieces
  /chess/vision/debug_image    — annotated image

Subscribes:
  /camera/image_raw
  /camera/camera_info
  /chess/arm_status
  /chess/last_move
"""

import json
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

import tf2_ros
from cv_bridge import CvBridge

FILES = 'abcdefgh'
ALL_SQUARES = [f + str(r) for r in range(1, 9) for f in FILES]
WHITE_START  = set([f + '1' for f in FILES] + [f + '2' for f in FILES])

# SDF colours for tile detection (HSV ranges, Gazebo-rendered)
# Light tiles: ambient 0.95 0.95 0.85 → creamy off-white
LIGHT_HSV_LO = np.array([40,  0, 160], np.uint8)
LIGHT_HSV_HI = np.array([150, 150, 255], np.uint8)
# Dark tiles:  ambient 0.15 0.10 0.05 → very dark brown
DARK_HSV_LO  = np.array([ 5, 40,  10], np.uint8)
DARK_HSV_HI  = np.array([70,180,  70], np.uint8)


class ChessVisionNode(Node):
    # States
    SEARCHING     = 'SEARCHING'      # trying to find board grid
    WAIT_EMPTY    = 'WAIT_EMPTY'     # board found, waiting for "Remove Pieces"
    CAPTURING_REF = 'CAPTURING_REF'  # collecting empty-board frames
    TRACKING      = 'TRACKING'       # live piece detection + move tracking

    REF_FRAMES_NEEDED = 15           # frames to average for empty-board ref

    def __init__(self):
        super().__init__('chess_vision_node')

        self.declare_parameter('origin_x',         0.20)
        self.declare_parameter('origin_y',         -0.175)
        self.declare_parameter('origin_z',          0.02)
        self.declare_parameter('square_size',       0.045)
        self.declare_parameter('sample_radius',     10)
        self.declare_parameter('change_threshold',  18.0)
        self.declare_parameter('piece_threshold',   22.0)

        self.ox          = self.get_parameter('origin_x').value
        self.oy          = self.get_parameter('origin_y').value
        self.oz          = self.get_parameter('origin_z').value
        self.sq          = self.get_parameter('square_size').value
        self.sample_r    = self.get_parameter('sample_radius').value
        self.change_thr  = self.get_parameter('change_threshold').value
        self.piece_thr   = self.get_parameter('piece_threshold').value

        self.bridge       = CvBridge()
        self.camera_info  = None
        self.arm_idle     = True
        self.last_move    = None

        # Vision state
        self.state          = self.SEARCHING
        self.grid_centres   = {}    # sq → (u, v)  — in-frame only
        self.empty_ref      = {}    # sq → mean gray brightness of empty square
        self.prev_idle_gray = {}    # sq → mean gray at last idle frame
        self.piece_sqs      = set() # squares currently occupied (diff vs empty_ref)
        self.changed_sqs    = set() # squares that changed since last idle frame

        # Ref capture accumulator
        self._ref_acc    = {}   # sq → list of brightness samples
        self._ref_count  = 0

        # Logging throttle
        self._log_counter  = 0
        self._log_interval = 30

        # TF (fallback projection)
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb,   10)
        self.create_subscription(Image,      '/camera/image_raw',   self._image_cb,   1)
        self.create_subscription(String,     '/chess/arm_status',   self._status_cb, 10)
        self.create_subscription(String,     '/chess/last_move',    self._move_cb,   10)
        self.create_subscription(String,     '/chess/cmd',          self._cmd_cb,    10)

        self.squares_pub = self.create_publisher(String, '/chess/vision/white_squares', 10)
        self.debug_pub   = self.create_publisher(Image,  '/chess/vision/debug_image',   10)

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
        prev = self.arm_idle
        self.arm_idle = msg.data in ('IDLE', 'DONE')
        if self.arm_idle != prev:
            self.get_logger().info(f'Arm status → {msg.data}')

    def _move_cb(self, msg):
        self.last_move = msg.data.strip()
        self.get_logger().info(f'Last move: {self.last_move}')

    def _cmd_cb(self, msg):
        cmd = msg.data.strip()
        if cmd == 'REMOVE_PIECES':
            if self.state in (self.WAIT_EMPTY, self.TRACKING, self.SEARCHING):
                self.state = self.CAPTURING_REF
                self._ref_acc   = {}
                self._ref_count = 0
                self.get_logger().info(
                    'REMOVE_PIECES received — entering CAPTURING_REF')
        elif cmd == 'RESET':
            self.state        = self.SEARCHING
            self.grid_centres = {}
            self.empty_ref    = {}
            self.piece_sqs    = set()
            self.changed_sqs  = set()
            self.get_logger().info('RESET — restarting board search')

    def _image_cb(self, msg):
        if self.camera_info is None:
            self._log_counter += 1
            if self._log_counter % self._log_interval == 1:
                self.get_logger().warn('Waiting for /camera/camera_info ...')
            return

        img  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        self._overlay_edges(img, gray)

        if self.state == self.SEARCHING:
            self._run_searching(img, gray, hsv)

        elif self.state == self.WAIT_EMPTY:
            self._draw_board_outline(img)
            self._draw_tile_grid(img)
            self._draw_square_markers(img)

        elif self.state == self.CAPTURING_REF:
            self._draw_board_outline(img)
            self._draw_tile_grid(img)
            if self.arm_idle:
                self._accumulate_ref(gray, img)

        elif self.state == self.TRACKING:
            self._run_tracking(img, gray)

        self._draw_hud(img)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))

    # ── State runners ─────────────────────────────────────────────────────────

    def _run_searching(self, img, gray, hsv):
        self._draw_tile_candidates(img, hsv)
        self._log_counter += 1

        found, method = False, None
        if self._detect_via_corners(gray):
            found, method = True, 'chessboard corners'
        elif self._detect_via_tiles(hsv):
            found, method = True, 'tile colour'
        elif self._detect_via_tf():
            found, method = True, 'TF projection'

        if found:
            n = len(self.grid_centres)
            self.get_logger().info(
                f'Board found via {method} — {n} in-frame squares')
            self.state = self.WAIT_EMPTY
        else:
            if self._log_counter % self._log_interval == 0:
                self.get_logger().info(
                    f'[SEARCHING] frame={self._log_counter}  '
                    f'arm_idle={self.arm_idle}')
            # Still show chessboard corner attempt
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            ret, corners = cv2.findChessboardCorners(gray, (7, 7), flags=flags)
            if ret:
                cv2.drawChessboardCorners(img, (7, 7), corners, ret)

    def _run_tracking(self, img, gray):
        if self.arm_idle and self.empty_ref:
            # Detect pieces as diff vs empty reference
            self.piece_sqs = set()
            for sq, uv in self.grid_centres.items():
                cur   = self._sample(gray, uv)
                empty = self.empty_ref.get(sq)
                if empty is not None and abs(cur - empty) > self.piece_thr:
                    self.piece_sqs.add(sq)

            # Detect changes vs previous idle frame
            self.changed_sqs = set()
            for sq, uv in self.grid_centres.items():
                prev = self.prev_idle_gray.get(sq)
                cur  = self._sample(gray, uv)
                if prev is not None and abs(cur - prev) > self.change_thr:
                    self.changed_sqs.add(sq)

            # Update previous idle snapshot
            self.prev_idle_gray = {
                sq: self._sample(gray, uv)
                for sq, uv in self.grid_centres.items()
            }

        self._draw_board_outline(img)
        self._draw_tile_grid(img)
        self._draw_square_markers(img)

        out = String()
        out.data = json.dumps(sorted(self.piece_sqs))
        self.squares_pub.publish(out)

        self._log_counter += 1
        if self._log_counter % self._log_interval == 0:
            self.get_logger().info(
                f'[TRACKING] frame={self._log_counter}  '
                f'grid={len(self.grid_centres)}  '
                f'pieces={len(self.piece_sqs)}  '
                f'changed={sorted(self.changed_sqs)}  '
                f'arm_idle={self.arm_idle}')

    # ── Board detection ───────────────────────────────────────────────────────

    # Method 1 — OpenCV findChessboardCorners (7×7 inner corners of 8×8 board)
    def _detect_via_corners(self, gray) -> bool:
        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
                 cv2.CALIB_CB_NORMALIZE_IMAGE  |
                 cv2.CALIB_CB_FAST_CHECK)
        ret, corners = cv2.findChessboardCorners(gray, (7, 7), flags=flags)
        if not ret:
            return False

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        corners  = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)
        corners  = corners.reshape(7, 7, 2)

        # corners[row][col] = pixel of inner corner between ranks (row+1,row+2)
        # and files (col, col+1).  Extrapolate outward to get square centres.
        # Spacing vectors from the corner grid:
        dr = (corners[1:] - corners[:-1]).mean(axis=(0, 1))  # rank direction
        dc = (corners[:, 1:] - corners[:, :-1]).mean(axis=(0, 1))  # file direction

        # Corner [0][0] is between squares (rank1,file_a) and (rank2,file_b).
        # Square centre at rank r, file f = corners[0][0] + (r-0.5)*dr + (f-0.5)*dc
        # Mapping: row index 0..6 = rank 1..7 inner corners
        origin = corners[0][0]   # inner corner at rank1/rank2, file_a/file_b

        for fi, f in enumerate(FILES):
            for ri in range(8):
                rank = str(ri + 1)
                # Centre of square (f, rank) relative to the corner grid
                u = origin[0] + (ri - 0.5) * dr[0] + (fi - 0.5) * dc[0]
                v = origin[1] + (ri - 0.5) * dr[1] + (fi - 0.5) * dc[1]
                self.grid_centres[f + rank] = (int(round(u)), int(round(v)))
        return True

    # Method 2 — colour tile detection
    def _detect_via_tiles(self, hsv) -> bool:
        light = cv2.inRange(hsv, LIGHT_HSV_LO, LIGHT_HSV_HI)
        dark  = cv2.inRange(hsv, DARK_HSV_LO,  DARK_HSV_HI)

        centres = []
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        for mask in (light, dark):
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < 60:
                    continue
                bx, by, bw, bh = cv2.boundingRect(c)
                if not (0.3 < bw / max(bh, 1) < 3.5):
                    continue
                M = cv2.moments(c)
                if M['m00'] == 0:
                    continue
                centres.append((M['m10'] / M['m00'], M['m01'] / M['m00'], area))

        if len(centres) < 6:
            return False

        # Try grid fitting first
        if self._fit_grid_to_centres(centres):
            return True

        # Fallback: use bounding box of all blobs → divide into 8×8
        return self._grid_from_bbox(centres)

    def _fit_grid_to_centres(self, centres) -> bool:
        """Fit an 8×8 grid to detected tile centres using median spacing."""
        pts = np.array([(c[0], c[1]) for c in centres])

        # Estimate grid spacing from nearest-neighbour distances
        dists = []
        for i, p in enumerate(pts):
            d = np.linalg.norm(pts - p, axis=1)
            d = d[d > 1]
            dists.append(d.min())
        spacing = float(np.median(dists))
        if spacing < 5:
            return False

        # Snap each point to the nearest grid node
        origin = pts.min(axis=0)
        snapped = {}
        for px, py in pts:
            col = int(round((px - origin[0]) / spacing))
            row = int(round((py - origin[1]) / spacing))
            if 0 <= col < 8 and 0 <= row < 8:
                sq = FILES[col] + str(8 - row)
                snapped[sq] = (int(origin[0] + col * spacing),
                               int(origin[1] + row * spacing))

        if len(snapped) < 6:
            return False

        self.grid_centres = snapped
        return True

    def _grid_from_bbox(self, centres) -> bool:
        """Fallback: divide bounding box of all tile blobs into an 8×8 grid."""
        pts = np.array([(c[0], c[1]) for c in centres])
        x1, y1 = pts.min(axis=0)
        x2, y2 = pts.max(axis=0)
        bw, bh = x2 - x1, y2 - y1
        if bw < 20 or bh < 20:
            return False
        sq_w = bw / 8.0
        sq_h = bh / 8.0
        snapped = {}
        for ri in range(8):
            for fi, f in enumerate(FILES):
                u = int(x1 + (fi + 0.5) * sq_w)
                v = int(y1 + (7 - ri + 0.5) * sq_h)
                snapped[f + str(ri + 1)] = (u, v)
        self.get_logger().info(
            f'Grid from bbox: ({x1:.0f},{y1:.0f})→({x2:.0f},{y2:.0f}) '
            f'sq={sq_w:.1f}×{sq_h:.1f}px')
        self.grid_centres = snapped
        return True

    # Method 3 — TF projection fallback
    def _detect_via_tf(self) -> bool:
        if self.camera_info is None:
            return False
        try:
            tf = self.tf_buffer.lookup_transform(
                'camera_link', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return False

        K  = self.camera_info.k
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        q  = tf.transform.rotation
        R  = self._quat_to_mat(q.x, q.y, q.z, q.w)
        t  = tf.transform.translation
        tx = np.array([t.x, t.y, t.z])

        iw = self.camera_info.width
        ih = self.camera_info.height

        self.get_logger().info(
            f'TF cam←base  t=({t.x:.3f},{t.y:.3f},{t.z:.3f})  '
            f'q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})  '
            f'K fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}  '
            f'img={iw}x{ih}',
            throttle_duration_sec=5.0)

        centres = {}
        behind = 0
        out_of_frame = 0
        for sq in ALL_SQUARES:
            wx, wy, wz = self._sq_world(sq)
            p = R @ np.array([wx, wy, wz]) + tx
            if p[2] <= 0:
                behind += 1
                continue
            u = int(fx * p[0] / p[2] + cx)
            v = int(fy * p[1] / p[2] + cy)
            if 0 <= u < iw and 0 <= v < ih:
                centres[sq] = (u, v)
            else:
                out_of_frame += 1

        # Log a sample of projections for the a-file to show where board lands
        sample_log = []
        for sq in ['a1', 'a2', 'a8', 'h1', 'h8', 'e4']:
            wx, wy, wz = self._sq_world(sq)
            p = R @ np.array([wx, wy, wz]) + tx
            if p[2] > 0:
                u = int(fx * p[0] / p[2] + cx)
                v = int(fy * p[1] / p[2] + cy)
                sample_log.append(f'{sq}→({u},{v})')
            else:
                sample_log.append(f'{sq}→behind')
        self.get_logger().info(
            f'TF projection: in_frame={len(centres)} behind={behind} '
            f'off_screen={out_of_frame}  samples: {" ".join(sample_log)}')

        if len(centres) < 4:
            return False
        self.grid_centres = centres
        return True

    # ── Reference capture + sampling ─────────────────────────────────────────

    def _accumulate_ref(self, gray, img):
        """Collect frames into empty-board reference average."""
        for sq, uv in self.grid_centres.items():
            self._ref_acc.setdefault(sq, []).append(self._sample(gray, uv))
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
            self.empty_ref = {sq: float(np.mean(v))
                              for sq, v in self._ref_acc.items()}
            self.prev_idle_gray = dict(self.empty_ref)
            self.piece_sqs   = set()
            self.changed_sqs = set()
            self.state = self.TRACKING
            self.get_logger().info(
                f'Empty board reference captured ({len(self.empty_ref)} squares) '
                f'— entering TRACKING')

    def _sample(self, gray, uv) -> float:
        u, v = uv
        h, w = gray.shape
        r    = self.sample_r
        patch = gray[max(0, v - r):min(h, v + r), max(0, u - r):min(w, u + r)]
        return float(np.mean(patch)) if patch.size > 0 else 0.0

    # ── Debug drawing layers ───────────────────────────────────────────────────

    def _overlay_edges(self, img, gray):
        """Tinted Canny edge overlay — shows board/tile boundaries."""
        edges = cv2.Canny(gray, 40, 120)
        edge_layer = np.zeros_like(img)
        edge_layer[edges > 0] = (180, 60, 0)   # dim blue-ish tint
        cv2.addWeighted(img, 1.0, edge_layer, 0.35, 0, img)

    def _draw_tile_candidates(self, img, hsv):
        """During SEARCHING: highlight light and dark tile colour candidates."""
        light = cv2.inRange(hsv, LIGHT_HSV_LO, LIGHT_HSV_HI)
        dark  = cv2.inRange(hsv, DARK_HSV_LO,  DARK_HSV_HI)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        light_count = 0
        dark_count  = 0
        for mask, color, label in (
                (light, (100, 220, 255), 'light'),   # BGR orange-yellow
                (dark,  (60,  60,  200), 'dark')):   # BGR red
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) < 80:
                    continue
                # Black thick shadow then coloured outline
                cv2.drawContours(img, [c], -1, (0, 0, 0), 6)
                cv2.drawContours(img, [c], -1, color,      3)
                if label == 'light':
                    light_count += 1
                else:
                    dark_count += 1

        h, w = img.shape[:2]
        info = f'SEARCHING  light={light_count}  dark={dark_count}  (need >=20 total)'
        cv2.putText(img, info, (8, h - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0,   0,   0), 4)
        cv2.putText(img, info, (8, h - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (200, 200, 200), 1)

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
        """Draw grid lines connecting adjacent square centres."""
        h, w = img.shape[:2]

        def in_frame(u, v):
            return 0 <= u < w and 0 <= v < h

        for ri in range(8):
            for fi in range(8):
                sq  = FILES[fi] + str(ri + 1)
                uv  = self.grid_centres.get(sq)
                if not uv or not in_frame(*uv):
                    continue
                # Draw line to right neighbour
                if fi < 7:
                    sq2 = FILES[fi + 1] + str(ri + 1)
                    uv2 = self.grid_centres.get(sq2)
                    if uv2 and in_frame(*uv2):
                        cv2.line(img, uv, uv2, (0, 0, 0),   3)
                        cv2.line(img, uv, uv2, (80, 80, 80), 1)
                # Draw line to upper neighbour
                if ri < 7:
                    sq2 = FILES[fi] + str(ri + 2)
                    uv2 = self.grid_centres.get(sq2)
                    if uv2 and in_frame(*uv2):
                        cv2.line(img, uv, uv2, (0, 0, 0),   3)
                        cv2.line(img, uv, uv2, (80, 80, 80), 1)

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
            self.TRACKING:      (0, 220,   0),   # green
        }.get(self.state, (200, 200, 200))

        cv2.rectangle(img, (0, 0), (w, 54), (0, 0, 0), -1)

        extra = ''
        if self.state == self.TRACKING:
            extra = f'  pieces={len(self.piece_sqs)}  changed={len(self.changed_sqs)}'
        elif self.state == self.CAPTURING_REF:
            extra = f'  {self._ref_count}/{self.REF_FRAMES_NEEDED} frames'

        label = f'{self.state}   grid={len(self.grid_centres)}{extra}'
        cv2.putText(img, label, (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0),   6)
        cv2.putText(img, label, (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.75, state_color, 2)

        # Centre instruction overlay
        cx, cy = w // 2, h // 2
        if self.state == self.SEARCHING:
            cv2.putText(img, 'BOARD NOT FOUND', (cx - 160, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,  0, 0), 8)
            cv2.putText(img, 'BOARD NOT FOUND', (cx - 160, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 80, 255), 3)
        elif self.state == self.WAIT_EMPTY:
            msg = 'Click  Remove Pieces  then wait'
            cv2.putText(img, msg, (cx - 200, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,   0,   0), 6)
            cv2.putText(img, msg, (cx - 200, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _sq_world(self, sq):
        fi = FILES.index(sq[0])
        ri = int(sq[1]) - 1
        return (self.ox + ri * self.sq + self.sq / 2,
                self.oy + fi * self.sq + self.sq / 2,
                self.oz + 0.035)

    @staticmethod
    def _quat_to_mat(x, y, z, w):
        return np.array([
            [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
            [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
            [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
        ], dtype=float)


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
