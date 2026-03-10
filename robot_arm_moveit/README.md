# robot_arm_moveit

ROS 2 Jazzy · MoveIt 2 · Gazebo Harmonic 8.x  
MoveIt 2 configuration + analytical/MoveIt IK for `robot_arm_description`.

---

## Package Structure

```
robot_arm_moveit/
├── config/
│   ├── robot_arm.srdf          # Planning groups, named poses, collision pairs
│   ├── kinematics.yaml         # KDL IK solver config
│   ├── ompl_planning.yaml      # OMPL planner (RRTConnect default)
│   ├── joint_limits.yaml       # Velocity/acceleration limits for planning
│   ├── moveit_controllers.yaml # Maps MoveIt groups to ros2_control controllers
│   └── moveit.rviz             # RViz layout with MotionPlanning panel
├── launch/
│   └── moveit.launch.py        # Starts move_group + RViz
├── scripts/
│   └── arm_ik.py               # Analytical IK + MoveIt Python client
├── CMakeLists.txt
└── package.xml
```

---

## Planning Groups

| Group    | Joints                                                    | IK tip  |
|----------|-----------------------------------------------------------|---------|
| arm      | base_yaw, shoulder_roll, shoulder_pitch, elbow_pitch      | tool0   |
| gripper  | finger_1_joint, finger_2_joint, finger_3_joint            | —       |

---

## Dependencies

```bash
sudo apt install \
  ros-jazzy-moveit \
  ros-jazzy-moveit-ros-planning-interface \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-moveit-kinematics \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-ros-visualization
```

---

## Build

```bash
cd ~/Desktop/Arm
colcon build --packages-select robot_arm_moveit --symlink-install
source install/setup.bash
```

---

## Running

### Step 1 — Start Gazebo simulation (terminal 1)
```bash
arm_sim
```

### Step 2 — Start MoveIt move_group + RViz (terminal 2)
```bash
ros2 launch robot_arm_moveit moveit.launch.py
```

In RViz, use the **MotionPlanning** panel:
- Drag the interactive marker (orange sphere at tool0) to a target pose
- Click **Plan** to preview the trajectory
- Click **Execute** to send it to Gazebo

---

## Analytical IK

Fast geometric solver — no MoveIt required, sends directly to `arm_controller`.

```bash
# Basic usage
ros2 run robot_arm_moveit arm_ik.py --x 0.2 --y 0.1 --z 0.4

# Elbow down configuration
ros2 run robot_arm_moveit arm_ik.py --x 0.2 --y 0.0 --z 0.3 --elbow down

# Custom duration
ros2 run robot_arm_moveit arm_ik.py --x 0.15 --y 0.15 --z 0.35 --duration 2.0
```

Output includes joint angles, FK verification, and position error in mm.

### Reachable workspace (approximate)
- Min reach: ~0.03 m from base
- Max reach: ~0.49 m from base  
- Height range: ~0.07 m to ~0.57 m
- Full rotation: ±180° around Z (base_yaw)

---

## MoveIt IK (requires move_group running)

```bash
ros2 run robot_arm_moveit arm_ik.py --mode moveit --x 0.2 --y 0.1 --z 0.4
```

---

## Named Poses (accessible from any MoveIt client)

| Name   | Group   | Description              |
|--------|---------|--------------------------|
| home   | arm     | All joints at zero       |
| reach  | arm     | Extended forward         |
| pick   | arm     | Pitched down to pick     |
| open   | gripper | All fingers open         |
| closed | gripper | All fingers closed 70°   |

---

## Changelog

### v0.4.0 — Initial MoveIt 2 integration
- New package `robot_arm_moveit` alongside `robot_arm_description`
- `robot_arm.srdf`: planning groups (arm + gripper), end effector, named poses, disabled collision pairs
- `kinematics.yaml`: KDL numerical IK solver
- `ompl_planning.yaml`: RRTConnect planner (default), RRT, PRM, LBKPIECE available
- `joint_limits.yaml`: velocity + acceleration limits for trajectory time parameterization
- `moveit_controllers.yaml`: bridges MoveIt to existing `arm_controller` + `gripper_controller`
- `moveit.launch.py`: launches `move_group` + RViz with MotionPlanning panel
- `arm_ik.py`: analytical geometric IK solver with FK verification + MoveIt client mode
  - Solves base_yaw from atan2(y,x)
  - Solves 3R planar IK using law of cosines on L2, L3+L4
  - Splits shoulder angle between shoulder_roll and shoulder_pitch
  - Elbow up/down configurations
  - Joint limit clamping with warnings
  - Position error reported in mm
