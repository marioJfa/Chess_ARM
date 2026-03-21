#!/usr/bin/env python3
"""
chess_coord_calibrator.py
Camera-based board coordinate calibration node.

Pipeline:
  1. On startup (after arm is IDLE at standby), or on ~/calibrate service call:
  2. Capture image + joint states from current standby position
  3. Build camera-to-world transform via forward kinematics + camera-on-wrist offset
  4. Back-project all 64 square centres through ray-plane intersection (board is flat table)
  5. Refine with ArUco ground-truth anchor correction (least-squares 2D similarity transform)
  6. Publish calibrated world coords to /chess/coord_calibrator/square_coords (JSON)

The arm does NOT move — calibration uses whatever pose it is currently in (standby).
ArUco markers must be visible from standby for refinement; falls back to pure FK if not.

Subscribes:
  /camera/image_raw    — wrist camera image
  /camera/camera_info  — camera intrinsics (K matrix)
  /joint_states        — current arm joint angles (for FK)
  /chess/arm_status    — wait for IDLE before auto-calibrating

Publishes:
  /chess/coord_calibrator/square_coords  — JSON {sq: [x, y, z], ...}
  /chess/coord_calibrator/status         — IDLE / CALIBRATING / DONE / ERROR

Service:
  ~/calibrate  (std_srvs/Trigger) — trigger re-calibration at current arm pose
"""

import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cv_bridge import CvBridge

# ── Import FK constants from robot_arm_moveit ────────────────────────────────
sys.path.insert(0, os.path.normpath(
    os.path.join(os.path.dirname(__file__),
                 '..', '..', '..', 'robot_arm_moveit', 'lib', 'robot_arm_moveit')))
from arm_ik import L1, L2, L3, L4  # noqa: E402

# ── ArUco setup — same dictionary as chess_vision_node ───────────────────────
ARUCO_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

FILES = 'abcdefgh'


class ChessCoordCalibratorNode(Node):

    STATUS_IDLE        = 'IDLE'
    STATUS_CALIBRATING = 'CALIBRATING'
    STATUS_DONE        = 'DONE'
    STATUS_ERROR       = 'ERROR'

    def __init__(self):
        super().__init__('chess_coord_calibrator')

        # ── Parameters ───────────────────────────────────────────────────────
        # Board geometry (shared with other nodes via board_config.yaml)
        self.declare_parameter('origin_x',    0.20)
        self.declare_parameter('origin_y',   -0.175)
        self.declare_parameter('origin_z',    0.02)
        self.declare_parameter('square_size', 0.045)
        self.declare_parameter('board_flip',  False)

        # Board surface Z in world frame (same as origin_z by default)
        self.declare_parameter('board_z', 0.02)

        # Physical anchor: arm base touches board near edge — offset in X (m)
        self.declare_parameter('board_edge_offset_x', 0.0)

        # Camera-on-wrist offset (matches URDF camera_joint: xyz=0.04 0 0.02)
        # Tune here if physical camera differs from URDF model
        self.declare_parameter('cam_offset_x', 0.04)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_offset_z', 0.02)

        # ArUco inner corner offset from board playing-area edge (squares)
        self.declare_parameter('aruco_inner_offset', 0.322)

        self._load_params()
        self.add_on_set_parameters_callback(self._on_param_change)

        # ── State ────────────────────────────────────────────────────────────
        self._bridge            = CvBridge()
        self._camera_info       = None
        self._latest_image      = None
        self._latest_joints     = {}    # joint_name → angle
        self._arm_status        = 'UNKNOWN'
        self._calibrated_coords = {}    # sq → [x, y, z]
        self._auto_calib_done   = False
        self._calib_lock        = threading.Lock()

        # ── Publishers ───────────────────────────────────────────────────────
        self._coords_pub = self.create_publisher(
            String, '/chess/coord_calibrator/square_coords', 10)
        self._status_pub = self.create_publisher(
            String, '/chess/coord_calibrator/status', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb,   10)
        self.create_subscription(Image,      '/camera/image_raw',   self._image_cb,  1)
        self.create_subscription(JointState, '/joint_states',       self._joints_cb, 10)
        self.create_subscription(String,     '/chess/arm_status',   self._status_cb, 10)

        # ── Service ──────────────────────────────────────────────────────────
        self.create_service(Trigger, '~/calibrate', self._calibrate_cb)

        self._publish_status(self.STATUS_IDLE)
        self.get_logger().info(
            '[CALIB] Chess coord calibrator ready — will calibrate when arm is IDLE')

    # ── Parameter helpers ────────────────────────────────────────────────────

    def _load_params(self):
        self.ox           = self.get_parameter('origin_x').value
        self.oy           = self.get_parameter('origin_y').value
        self.oz           = self.get_parameter('origin_z').value
        self.sq           = self.get_parameter('square_size').value
        self.board_flip   = self.get_parameter('board_flip').value
        self.board_z      = self.get_parameter('board_z').value
        self.edge_offset  = self.get_parameter('board_edge_offset_x').value
        self.cam_dx       = self.get_parameter('cam_offset_x').value
        self.cam_dy       = self.get_parameter('cam_offset_y').value
        self.cam_dz       = self.get_parameter('cam_offset_z').value
        self.aruco_offset = self.get_parameter('aruco_inner_offset').value

    def _on_param_change(self, params):
        self._load_params()
        return SetParametersResult(successful=True)

    # ── ROS callbacks ────────────────────────────────────────────────────────

    def _info_cb(self, msg: CameraInfo):
        if self._camera_info is None:
            self.get_logger().info(
                f'[CALIB] Camera info: {msg.width}x{msg.height}  '
                f'fx={msg.k[0]:.1f} fy={msg.k[4]:.1f}  '
                f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')
        self._camera_info = msg

    def _image_cb(self, msg: Image):
        self._latest_image = msg

    def _joints_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._latest_joints[name] = pos

    def _status_cb(self, msg: String):
        prev             = self._arm_status
        self._arm_status = msg.data
        # Auto-calibrate on first IDLE after startup
        if prev not in ('IDLE', 'DONE') and self._arm_status in ('IDLE', 'DONE'):
            if not self._auto_calib_done:
                self.get_logger().info(
                    '[CALIB] Arm reached IDLE — triggering auto-calibration at standby')
                threading.Thread(target=self._run_calibration, daemon=True).start()

    def _calibrate_cb(self, req: Trigger.Request, res: Trigger.Response):
        self.get_logger().info('[CALIB] ~/calibrate service called')
        ok, msg       = self._run_calibration()
        res.success   = ok
        res.message   = msg
        return res

    # ── Core calibration pipeline ─────────────────────────────────────────────

    def _run_calibration(self) -> tuple:
        """Calibrate using the current arm pose (no movement). Thread-safe."""
        if not self._calib_lock.acquire(blocking=False):
            return False, 'Calibration already in progress'

        try:
            self._auto_calib_done = True
            self._publish_status(self.STATUS_CALIBRATING)

            # 1. Capture snapshot at current pose (arm is at standby)
            image, cam_info, joints = self._snapshot()
            if image is None or cam_info is None:
                self._publish_status(self.STATUS_ERROR)
                return False, 'Snapshot failed — no camera/joint data available'

            # 2. Compute square world coordinates
            coords = self._compute_square_coords(image, cam_info, joints)
            if not coords:
                self._publish_status(self.STATUS_ERROR)
                return False, 'Coordinate computation failed'

            # 3. Publish and cache
            self._calibrated_coords = coords
            self._publish_coords(coords)
            self._publish_status(self.STATUS_DONE)
            self.get_logger().info(
                f'[CALIB] Calibration complete — {len(coords)} squares computed')
            return True, f'Calibrated {len(coords)} squares'

        except Exception as e:
            self.get_logger().error(f'[CALIB] Calibration error: {e}')
            self._publish_status(self.STATUS_ERROR)
            return False, str(e)
        finally:
            self._calib_lock.release()

    # ── Data capture ─────────────────────────────────────────────────────────

    def _snapshot(self, timeout: float = 3.0) -> tuple:
        """Grab the latest image, camera_info, and joint_states.

        Returns (cv2_image, CameraInfo, joints_dict) or (None, None, None) on timeout.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if (self._latest_image is not None
                    and self._camera_info is not None
                    and self._latest_joints):
                try:
                    cv_img      = self._bridge.imgmsg_to_cv2(
                        self._latest_image, desired_encoding='bgr8')
                    joints_snap = dict(self._latest_joints)
                    cam_snap    = self._camera_info
                    return cv_img, cam_snap, joints_snap
                except Exception as e:
                    self.get_logger().warn(f'[CALIB] Image conversion error: {e}')
            time.sleep(0.05)
        self.get_logger().warn('[CALIB] Snapshot timeout — no camera/joint data')
        return None, None, None

    # ── ArUco detection ───────────────────────────────────────────────────────

    def _detect_aruco(self, image: np.ndarray) -> dict:
        """Detect ArUco markers. Returns {marker_id: corners_4x2_px}."""
        gray            = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
        result = {}
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                result[int(mid)] = corners[i][0]   # shape (4, 2)
        return result

    # ── Coordinate computation ────────────────────────────────────────────────

    def _compute_square_coords(self,
                               image: np.ndarray,
                               cam_info: CameraInfo,
                               joints: dict) -> dict:
        """FK → camera transform → base tile coords → ArUco refinement.

        Returns {sq_name: [x, y, z]} for all 64 squares.
        """
        # 1. Build camera world transform from current joints + cam offset
        T_world_cam = self._build_camera_transform(joints)
        if T_world_cam is None:
            self.get_logger().error('[CALIB] FK failed — cannot build camera transform')
            return {}

        cam_pos = T_world_cam[:3, 3]
        R_wc    = T_world_cam[:3, :3]

        self.get_logger().info(
            f'[CALIB] Camera world pos: '
            f'x={cam_pos[0]:.3f}  y={cam_pos[1]:.3f}  z={cam_pos[2]:.3f}')

        # 2. Camera intrinsics
        K  = np.array(cam_info.k, dtype=np.float64).reshape(3, 3)
        Ki = np.linalg.inv(K)

        # 3. Base tile-centre coords (FK-derived board geometry)
        raw_coords = {}
        for ri in range(8):
            for fi in range(8):
                sq_name = f'{FILES[fi]}{ri + 1}'
                if self.board_flip:
                    wx = self.ox + (7 - ri + 0.5) * self.sq
                    wy = self.oy + (7 - fi + 0.5) * self.sq
                else:
                    wx = self.ox + (ri + 0.5) * self.sq
                    wy = self.oy + (fi + 0.5) * self.sq
                raw_coords[sq_name] = [round(wx, 4), round(wy, 4), round(self.board_z, 4)]

        # 4. Detect ArUco markers for refinement
        detected = self._detect_aruco(image)
        self.get_logger().info(
            f'[CALIB] ArUco markers detected: {list(detected.keys())}')

        if len(detected) >= 2:
            refined = self._refine_with_aruco(raw_coords, detected, Ki, R_wc, cam_pos)
            if refined:
                return refined
            self.get_logger().warn(
                '[CALIB] Refinement failed — using raw FK-based tile centres')
        else:
            self.get_logger().warn(
                f'[CALIB] Only {len(detected)} ArUco marker(s) visible from standby — '
                f'using FK-only coords (no ArUco refinement)')

        return raw_coords

    def _build_camera_transform(self, joints: dict):
        """Build 4×4 T_world_camera from current joint angles + fixed camera-on-wrist offset.

        Returns 4×4 numpy array, or None on error.
        """
        try:
            yaw = joints.get('base_yaw',       0.0)
            sr  = joints.get('shoulder_roll',  0.0)
            sp  = joints.get('shoulder_pitch', 0.0)
            ep  = joints.get('elbow_pitch',    0.0)

            shoulder_z  = 0.06 + L1          # world Z of shoulder_roll joint
            total_pitch = sr + sp + ep

            # Tool0 position in world frame (same as arm_ik.forward_kinematics)
            r    = L2 * math.cos(sr + sp) + (L3 + L4) * math.cos(total_pitch)
            t0_z = shoulder_z + L2 * math.sin(sr + sp) + (L3 + L4) * math.sin(total_pitch)
            t0_x = r * math.cos(yaw)
            t0_y = r * math.sin(yaw)

            # Tool0 orientation: forward axis (along wrist) in world frame
            fwd_x = math.cos(total_pitch) * math.cos(yaw)
            fwd_y = math.cos(total_pitch) * math.sin(yaw)
            fwd_z = math.sin(total_pitch)

            # Right axis: perpendicular to fwd in XY plane
            right_x = -math.sin(yaw)
            right_y =  math.cos(yaw)
            right_z =  0.0

            # Up axis: right × fwd
            up_x = fwd_y * right_z - fwd_z * right_y
            up_y = fwd_z * right_x - fwd_x * right_z
            up_z = fwd_x * right_y - fwd_y * right_x

            # Rotation matrix: columns = tool0 X, Y, Z in world frame
            R_tool0 = np.array([
                [right_x, up_x, fwd_x],
                [right_y, up_y, fwd_y],
                [right_z, up_z, fwd_z],
            ])

            # Apply camera-on-wrist offset (in tool0 frame, URDF: xyz=0.04 0 0.02)
            cam_offset = np.array([self.cam_dx, self.cam_dy, self.cam_dz])
            cam_pos    = np.array([t0_x, t0_y, t0_z]) + R_tool0 @ cam_offset

            T = np.eye(4)
            T[:3, :3] = R_tool0
            T[:3,  3] = cam_pos
            return T

        except Exception as e:
            self.get_logger().error(f'[CALIB] _build_camera_transform error: {e}')
            return None

    def _pixel_to_world(self,
                        u: float, v: float,
                        Ki: np.ndarray,
                        R_wc: np.ndarray,
                        cam_pos: np.ndarray) -> np.ndarray:
        """Back-project pixel (u, v) to world XYZ via ray–plane intersection.

        Board is a horizontal plane at z = self.board_z in world frame.
        Returns world point or None if ray is parallel to plane / behind camera.
        """
        ray_w = R_wc @ (Ki @ np.array([u, v, 1.0]))
        if abs(ray_w[2]) < 1e-6:
            return None
        t = (self.board_z - cam_pos[2]) / ray_w[2]
        if t < 0:
            return None
        return cam_pos + t * ray_w

    # ── ArUco-based refinement ────────────────────────────────────────────────

    def _refine_with_aruco(self,
                           raw_coords: dict,
                           detected: dict,
                           Ki: np.ndarray,
                           R_wc: np.ndarray,
                           cam_pos: np.ndarray) -> dict:
        """Fit a 2D similarity transform from detected ArUco anchors → known world positions.

        For each detected marker, back-projects its inner corner pixel to world XY,
        then computes the least-squares correction (Umeyama) against the known board geometry.
        Applies the correction to all 64 square coords.

        Returns corrected coords dict, or empty dict on failure.
        """
        # Inner corner index per marker ID (same as chess_vision_node)
        inner_idx = {0: 1, 1: 0, 2: 2, 3: 3}
        _D = self.aruco_offset  # inner-corner offset from board edge in squares

        # Known world XY of each marker's inner corner
        known_world = {
            0: (self.ox + (-_D + 0.5) * self.sq, self.oy + (-_D + 0.5) * self.sq),
            1: (self.ox + (8 + _D - 0.5) * self.sq, self.oy + (-_D + 0.5) * self.sq),
            2: (self.ox + (-_D + 0.5) * self.sq, self.oy + (8 + _D - 0.5) * self.sq),
            3: (self.ox + (8 + _D - 0.5) * self.sq, self.oy + (8 + _D - 0.5) * self.sq),
        }

        src_pts, dst_pts = [], []
        for mid, corners_px in detected.items():
            if mid not in inner_idx or mid not in known_world:
                continue
            px    = corners_px[inner_idx[mid]]
            world = self._pixel_to_world(px[0], px[1], Ki, R_wc, cam_pos)
            if world is None:
                continue
            src_pts.append([world[0], world[1]])
            dst_pts.append(list(known_world[mid]))

        if len(src_pts) < 2:
            self.get_logger().warn(
                f'[CALIB] Only {len(src_pts)} usable ArUco anchor(s) — need ≥2 for refinement')
            return {}

        src = np.array(src_pts, dtype=np.float64)
        dst = np.array(dst_pts, dtype=np.float64)
        T2d, scale = self._umeyama_2d(src, dst)

        self.get_logger().info(
            f'[CALIB] ArUco refinement: {len(src_pts)} anchors  '
            f'scale={scale:.4f}  '
            f'tx={T2d[0, 2]:.4f}  ty={T2d[1, 2]:.4f}')

        corrected = {}
        for sq, (wx, wy, wz) in raw_coords.items():
            pt = T2d @ np.array([wx, wy, 1.0])
            corrected[sq] = [round(float(pt[0]), 4), round(float(pt[1]), 4), wz]
        return corrected

    @staticmethod
    def _umeyama_2d(src: np.ndarray, dst: np.ndarray) -> tuple:
        """Least-squares 2D similarity transform (scale * R + t) mapping src → dst.

        Returns (3×3 homogeneous T, scale).
        """
        n      = src.shape[0]
        mu_src = src.mean(axis=0)
        mu_dst = dst.mean(axis=0)
        src_c  = src - mu_src
        dst_c  = dst - mu_dst
        var_src = np.mean(np.sum(src_c ** 2, axis=1))
        cov     = (dst_c.T @ src_c) / n
        U, S, Vt = np.linalg.svd(cov)
        d  = np.sign(np.linalg.det(U @ Vt))
        D  = np.diag([1.0, d])
        R  = U @ D @ Vt
        sc = np.sum(S * np.array([1.0, d])) / max(var_src, 1e-10)
        t  = mu_dst - sc * R @ mu_src
        T  = np.eye(3)
        T[:2, :2] = sc * R
        T[:2,  2] = t
        return T, sc

    # ── Publishing ───────────────────────────────────────────────────────────

    def _publish_coords(self, coords: dict):
        msg      = String()
        msg.data = json.dumps(coords)
        self._coords_pub.publish(msg)
        self.get_logger().info(
            f'[CALIB] Published calibrated coords for {len(coords)} squares')

    def _publish_status(self, status: str):
        msg      = String()
        msg.data = status
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ChessCoordCalibratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
