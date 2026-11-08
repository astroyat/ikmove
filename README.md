# ikmove
Simple move control routines for <a href="https://github.com/TheRobotStudio/SO-ARM100">SO-ARM100</a> using <a href="https://github.com/Phylliade/ikpy">ikpy</a> for inverse kinematics and cubic polynomial for trajectory planning.

This implementation uses `arm.urdf` based on `SO_5DOF_ARM100_8j_URDF.SLDASM.urdf` from SO-ARM100.
```bash
git submodule update --init
```

## Standard SO-100 / SO-101 Joint Convention

`ikmove` follows the official **LeRobot standard SO-100 / SO-101 joint angle conventions**:

| Joint Index | Motor Name | $0^\circ$ Position | $+90^\circ$ Position | Nominal Range |
|:---:|:---|:---|:---|:---:|
| `0` | `shoulder_pan` | Straight forward | Facing left ($+90^\circ$) | $[-180^\circ, 180^\circ]$ |
| `1` | `shoulder_lift` | Horizontal forward | Vertical upright (candle pose) | $[0^\circ, 180^\circ]$ |
| `2` | `elbow_flex` | Straight in-line with upper arm | Bent at $90^\circ$ right angle | $[0^\circ, 180^\circ]$ |
| `3` | `wrist_flex` | Straight in-line with forearm | Bent down at $90^\circ$ right angle | $[-90^\circ, 90^\circ]$ |
| `4` | `wrist_roll` | Neutral rotation | Rotated $+90^\circ$ | $[-180^\circ, 180^\circ]$ |
| `5` | `gripper` | Fully closed | Open | $[0^\circ, 100^\circ]$ |

### Standard Postures
- **Upright "Candle" Pose**: `[0, 90, 0, 0, 0, 0]` (servos at `[2048, 2048, 1024, 1024, 3072, 1024]`).
- **Standard Zero Pose**: `[0, 0, 0, 0, 0, 0]` (servos at `[2048, 3072, 1024, 1024, 3072, 1024]`).

## Calibration & Servo Conversion

`joint_to_servo(joint)` and `servo_to_joint(servo)` apply standard LeRobot `revert_calibration` and `apply_calibration` formulas with `homing_offset` and `drive_mode` from `main_follower.json`:
- `homing_offset`: `[-2048, 3072, -1024, -1024, 3072, -1024]`
- `drive_mode`: `[0, 1, 0, 0, 1, 0]`

## Usage

```python
from ikmove import Ikmove

ik = Ikmove()

# Forward Kinematics: standard degrees -> Cartesian position
q, pos = ik.joint_to_pos([0, 90, 0, 0, 0, 0])
end_pos = pos[7][:3, 3]  # End-effector [x, y, z] in meters

# Inverse Kinematics: Cartesian position -> standard degrees
q, joint = ik.pos_to_joint(end_pos)

# Servo conversion: standard degrees -> Feetech STS3215 raw ticks [0..4095]
servos = ik.joint_to_servo([0, 90, 0, 0, 0, 0])
recovered_joint = ik.servo_to_joint(servos)

# Cycle through all SO-101 standard postures:
# rest -> homing -> candle -> zero -> rotated -> candle -> rest
waypoints = ik.get_standard_cycle_waypoints()
times = [150] * (len(waypoints) - 1)
trajectory = ik.joint_trajectory(waypoints, times)
ik.plot_trajectory(waypoints, times)
```

Note that in the URDF coordinate frame, the arm reaches forward in the negative y direction.


