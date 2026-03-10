#!/usr/bin/env python3
"""
arm_ik.py  —  v0.4.0
Analytical + MoveIt IK for robot_arm_description.

Provides two modes:
  1. ANALYTICAL  — fast geometric IK, no MoveIt needed, direct joint solution
  2. MOVEIT      — uses MoveGroup action server, full collision-aware planning

The arm chain (all joints pitch on Y except base_yaw on Z):
  base_yaw      (Z)  : rotates the whole arm around vertical
  shoulder_roll (Y)  : pitch at base of upper arm
  shoulder_pitch(Y)  : pitch at top of upper arm
  elbow_pitch   (Y)  : pitch at forearm

Link lengths (from URDF):
  L1 = 0.04  m  (base to shoulder_roll)
  L2 = 0.20  m  (upper arm)
  L3 = 0.18  m  (forearm)
  L4 = 0.11  m  (wrist to tool0)

Usage (analytical):
  python3 arm_ik.py --mode analytical --x 0.2 --y 0.1 --z 0.4

Usage (moveit, requires Gazebo running):
  python3 arm_ik.py --mode moveit --x 0.2 --y 0.1 --z 0.4
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ── Arm geometry (metres) ────────────────────────────────────────────────────
L1 = 0.04   # base_link top → shoulder_roll joint
L2 = 0.20   # upper_arm length
L3 = 0.18   # forearm length
L4 = 0.11   # wrist → tool0

ARM_JOINTS = ['base_yaw', 'shoulder_roll', 'shoulder_pitch', 'elbow_pitch']

JOINT_LIMITS = {
    'base_yaw':       (-math.pi,       math.pi),
    'shoulder_roll':  (-0.7854,        0.7854),
    'shoulder_pitch': (-1.5708,        2.3562),
    'elbow_pitch':    (-2.0944,        2.0944),
}


# ── Analytical IK ─────────────────────────────────────────────────────────────
def analytical_ik(x: float, y: float, z: float, elbow_up: bool = True):
    """
    Geometric IK for the 4-DOF arm.

    Coordinate convention (ROS/URDF):
      X = forward, Y = left, Z = up
      Base sits at origin, arm extends upward along Z at rest.

    Strategy:
      1. base_yaw  = atan2(y, x)           — point arm toward target in XY
      2. Project target into the arm's vertical plane (r, z_eff)
      3. Solve 3R planar IK for shoulder_roll + shoulder_pitch + elbow_pitch
         using the law of cosines on L2, L3, L4

    Returns dict of joint_name → angle (radians), or None if unreachable.
    """

    # 1. Base yaw — rotate to face target
    base_yaw = math.atan2(y, x)

    # 2. Radial distance in XY plane
    r = math.sqrt(x**2 + y**2)

    # 3. Height above shoulder_roll joint origin
    #    shoulder_roll sits at z = L1 above base_link top (which is at z=0.06)
    #    but we work relative to shoulder_roll joint origin
    shoulder_z = 0.06 + L1   # world Z of shoulder_roll joint
    z_eff = z - shoulder_z   # height target relative to shoulder_roll

    # 4. Distance from shoulder_roll to tool0
    D = math.sqrt(r**2 + z_eff**2)

    # Reachability check
    reach_max = L2 + L3 + L4
    reach_min = abs(L2 - (L3 + L4))
    if D > reach_max:
        print(f'[IK] Target unreachable: distance {D:.3f}m > max reach {reach_max:.3f}m')
        return None
    if D < reach_min:
        print(f'[IK] Target too close: distance {D:.3f}m < min reach {reach_min:.3f}m')
        return None

    # 5. Reduce to 2R problem: treat (L3+L4) as one segment for elbow
    #    We solve for the wrist position first (tool0 - L4 along approach direction)
    #    Simple approach: solve with L2 and (L3+L4) combined reach
    L34 = L3 + L4

    # Law of cosines: D² = L2² + L34² - 2·L2·L34·cos(π - elbow_pitch)
    cos_elbow = (L2**2 + L34**2 - D**2) / (2 * L2 * L34)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))  # clamp for float errors
    elbow_angle = math.acos(cos_elbow)           # always positive
    elbow_pitch = math.pi - elbow_angle          # convert to joint angle

    if not elbow_up:
        elbow_pitch = -elbow_pitch

    # Shoulder angle: angle to target + offset from triangle
    alpha = math.atan2(z_eff, r)
    cos_beta = (L2**2 + D**2 - L34**2) / (2 * L2 * D)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)

    shoulder_total = alpha + beta if elbow_up else alpha - beta
    # Split shoulder_total between shoulder_roll and shoulder_pitch evenly
    shoulder_roll  = shoulder_total * 0.4
    shoulder_pitch = shoulder_total * 0.6

    solution = {
        'base_yaw':       base_yaw,
        'shoulder_roll':  shoulder_roll,
        'shoulder_pitch': shoulder_pitch,
        'elbow_pitch':    elbow_pitch,
    }

    # Check limits
    for joint, angle in solution.items():
        lo, hi = JOINT_LIMITS[joint]
        if not (lo <= angle <= hi):
            print(f'[IK] Joint {joint} = {math.degrees(angle):.1f}° out of limits '
                  f'[{math.degrees(lo):.1f}°, {math.degrees(hi):.1f}°]')
            # Clamp — caller can decide whether to use it
            solution[joint] = max(lo, min(hi, angle))

    return solution


def forward_kinematics(joints: dict) -> tuple:
    """
    Simple FK to verify IK solution.
    Returns (x, y, z) of tool0 in world frame.
    """
    yaw = joints['base_yaw']
    sr  = joints['shoulder_roll']
    sp  = joints['shoulder_pitch']
    ep  = joints['elbow_pitch']

    shoulder_z = 0.06 + L1
    total_pitch = sr + sp + ep

    # Radial reach in the arm's vertical plane
    r = L2 * math.cos(sr + sp) + (L3 + L4) * math.cos(total_pitch)
    z = shoulder_z + L2 * math.sin(sr + sp) + (L3 + L4) * math.sin(total_pitch)

    x = r * math.cos(yaw)
    y = r * math.sin(yaw)

    return (x, y, z)


# ── ROS node — sends IK solution to arm_controller ───────────────────────────
class IKNode(Node):

    def __init__(self):
        super().__init__('arm_ik_node')
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

    def send_solution(self, solution: dict, duration_sec: float = 3.0):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [solution[j] for j in ARM_JOINTS]
        pt.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        msg.points = [pt]
        self.arm_pub.publish(msg)
        self.get_logger().info(
            f'Sent IK solution: ' +
            ', '.join(f'{j}={math.degrees(v):.1f}°'
                      for j, v in solution.items())
        )


# ── MoveIt client (requires move_group running) ───────────────────────────────
def moveit_ik(node: Node, x: float, y: float, z: float):
    """
    Uses MoveIt MoveGroup action to plan and execute to a Cartesian pose.
    Requires: ros2 launch robot_arm_moveit moveit.launch.py
    """
    try:
        from moveit.python_bindings import MoveItPy
        from moveit.core.robot_state import RobotState
        from geometry_msgs.msg import Pose
    except ImportError:
        node.get_logger().error(
            'moveit Python bindings not found. '
            'Install: sudo apt install ros-jazzy-moveit-py')
        return

    node.get_logger().info('Connecting to MoveIt MoveGroup...')
    robot = MoveItPy(node_name='moveit_ik_client')
    arm   = robot.get_planning_component('arm')

    target = Pose()
    target.position.x = x
    target.position.y = y
    target.position.z = z
    target.orientation.w = 1.0  # neutral orientation

    arm.set_goal_state(pose_stamped_msg=target, pose_link='tool0')
    plan = arm.plan()

    if plan:
        node.get_logger().info('MoveIt plan found — executing...')
        robot.execute(plan, controllers=['arm_controller'])
    else:
        node.get_logger().error('MoveIt could not find a plan to target.')


# ── CLI ───────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Robot Arm IK')
    parser.add_argument('--mode',    choices=['analytical', 'moveit'],
                        default='analytical')
    parser.add_argument('--x',       type=float, default=0.2)
    parser.add_argument('--y',       type=float, default=0.0)
    parser.add_argument('--z',       type=float, default=0.4)
    parser.add_argument('--elbow',   choices=['up', 'down'], default='up')
    parser.add_argument('--duration',type=float, default=3.0,
                        help='Trajectory duration in seconds')
    args = parser.parse_args()

    print(f'\n[IK] Target: x={args.x}  y={args.y}  z={args.z}')
    print(f'[IK] Mode: {args.mode}  |  Elbow: {args.elbow}')

    rclpy.init()
    node = IKNode()

    if args.mode == 'analytical':
        solution = analytical_ik(args.x, args.y, args.z,
                                 elbow_up=(args.elbow == 'up'))
        if solution:
            fk_x, fk_y, fk_z = forward_kinematics(solution)
            print(f'[IK] Solution found:')
            for j, v in solution.items():
                print(f'       {j:<20} {math.degrees(v):>8.2f}°  ({v:.4f} rad)')
            print(f'[IK] FK verification: x={fk_x:.3f}  y={fk_y:.3f}  z={fk_z:.3f}')
            err = math.sqrt((fk_x-args.x)**2 + (fk_y-args.y)**2 + (fk_z-args.z)**2)
            print(f'[IK] Position error: {err*1000:.1f} mm')
            node.send_solution(solution, args.duration)
            # Give publisher time to send
            import time; time.sleep(0.5)
        else:
            print('[IK] No solution found.')

    elif args.mode == 'moveit':
        moveit_ik(node, args.x, args.y, args.z)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
