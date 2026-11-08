import json
from pathlib import Path
import numpy as np

URDF_PATH = Path(__file__).parent / "arm.urdf"
CALIB_PATH = Path(__file__).parent.parent.parent / "main_follower.json"

DEFAULT_HOMING_OFFSET = [-2048, 3072, -1024, -1024, 3072, -1024]
DEFAULT_DRIVE_MODE = [0, 1, 0, 0, 1, 0]


class Ikmove:
    def __init__(self, urdf_path=None, calib_path=None):
        self.ikpy_chain = None
        self.joint = [0, 0, 0, 0, 0, 0]
        self.urdf_path = str(urdf_path) if urdf_path else str(URDF_PATH)

        self.homing_offset = np.array(DEFAULT_HOMING_OFFSET, dtype=float)
        self.drive_mode = np.array(DEFAULT_DRIVE_MODE, dtype=int)

        calib_file = Path(calib_path) if calib_path else CALIB_PATH
        if not calib_file.is_file():
            calib_file = Path(__file__).parent / "main_follower.json"

        if calib_file.is_file():
            try:
                with open(calib_file) as f:
                    calib_data = json.load(f)
                if "homing_offset" in calib_data and "drive_mode" in calib_data:
                    self.homing_offset = np.array(calib_data["homing_offset"], dtype=float)
                    self.drive_mode = np.array(calib_data["drive_mode"], dtype=int)
            except Exception:
                pass

    def set_ikpy(self):
        from ikpy.chain import Chain

        self.ikpy_chain = Chain.from_urdf_file(
            urdf_file=self.urdf_path,
            active_links_mask=[False, True, True, True, True, True, False, False],
        )

    def joint_to_pos(self, joint):
        """Convert standard joint degrees to URDF link coordinates and FK pos."""
        if self.ikpy_chain is None:
            self.set_ikpy()

        q_deg = np.zeros(len(joint) + 2)
        q_deg[1] = joint[0]
        q_deg[2] = joint[1]
        q_deg[3] = joint[2]
        q_deg[4] = joint[3] - 90.0
        q_deg[5] = joint[4]
        q = np.radians(q_deg)

        pos = self.ikpy_chain.forward_kinematics(
            q[: len(self.ikpy_chain.links)], full_kinematics=True
        )
        return q, pos

    def pos_to_joint(self, pos, rot=None, mode=None):
        """Convert Cartesian position to joint degrees via IK."""
        if self.ikpy_chain is None:
            self.set_ikpy()

        q = self.ikpy_chain.inverse_kinematics(pos)
        if mode is not None:
            q = self.ikpy_chain.inverse_kinematics(
                pos,
                target_orientation=rot,
                orientation_mode=mode,
                initial_position=q,
            )

        joint = np.zeros(6)
        q_deg = np.degrees(q)
        joint[0] = q_deg[1]
        joint[1] = q_deg[2]
        joint[2] = q_deg[3]
        joint[3] = q_deg[4] + 90.0
        joint[4] = q_deg[5]
        joint[5] = self.joint[5]
        return q, joint

    def joint_to_servo(self, joint):
        """Convert joint degrees to raw Feetech servo ticks [0..4095] (2048 center)."""
        q = np.zeros(len(joint))
        q[0] = joint[0]
        q[1] = joint[1]
        q[2] = joint[2]
        q[3] = joint[3] - 90.0
        q[4] = joint[4]
        q[5] = joint[5]
        return np.round(2048.0 + ((q / 180.0) * 2048.0)).astype(int)

    def servo_to_joint(self, servo):
        """Convert raw Feetech servo ticks [0..4095] to joint degrees."""
        q_deg = (np.array(servo, dtype=float) - 2048.0) / 2048.0 * 180.0
        joint = np.zeros(len(q_deg))
        joint[0] = q_deg[0]
        joint[1] = q_deg[1]
        joint[2] = q_deg[2]
        joint[3] = q_deg[3] + 90.0
        joint[4] = q_deg[4]
        joint[5] = q_deg[5]
        return joint

    def plot(self, q, pos, title=None, ax=None, block=True):
        import ikpy.utils.plot as plot_utils
        import matplotlib.pyplot as plt

        show_at_end = False
        if ax is None:
            _, ax = plot_utils.init_3d_figure()
            show_at_end = True

        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.6, 0.2)
        ax.set_zlim(0, 0.6)

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        if title:
            ax.set_title(title, fontsize=10, fontweight="bold")

        end_fk = pos[7]
        end_pos = end_fk[:3, 3]
        self.ikpy_chain.plot(q, ax, end_pos)

        if show_at_end:
            plt.show(block=block)
            plt.close()

    def plot_deg(self, deg, name=None, block=True):
        servo = self.joint_to_servo(deg)
        q, pos = self.joint_to_pos(deg)
        end_fk = pos[7]
        end_pos = end_fk[:3, 3]
        end_rot = end_fk[:3, :3]

        pos_str = f"[{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}] m"
        title = f"{name.upper() if name else 'Pose'}\nDeg: {deg}\nServos: {servo.tolist()}\nXYZ: {pos_str}"
        print(f"=== {name.upper() if name else 'Pose'} ===")
        print("FK joint :", deg)
        print("FK servo :", servo.tolist())
        print("FK pos   :", pos_str)
        self.plot(q, pos, title=title, block=block)

    def plot_standard_positions(self, block=True):
        """Display all standard SO-101 positions side-by-side in a 3D GUI window."""
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(16, 5))
        fig.suptitle("SO-101 Standard Postures (3D Forward Kinematics)", fontsize=13, fontweight="bold")
        poses = self.get_standard_positions()

        for i, (name, deg) in enumerate(poses.items()):
            ax = fig.add_subplot(1, len(poses), i + 1, projection="3d")
            q, pos = self.joint_to_pos(deg)
            servo = self.joint_to_servo(deg)
            xyz = pos[7][:3, 3]
            sub_title = f"{name.upper()}\n{deg[:3]}\n{deg[3:]}\nXYZ: {[round(float(c), 2) for c in xyz]}"
            self.plot(q, pos, title=sub_title, ax=ax)

        plt.tight_layout()
        plt.show(block=block)
        plt.close()

    def animate_trajectory(self, joint_waypoints, times, interval=30):
        """Animate the 3D robot arm moving through the trajectory."""
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        traj = self.joint_trajectory(joint_waypoints, times)
        joint_positions = traj["joint_positions"]

        fig = plt.figure(figsize=(8, 7))
        ax = fig.add_subplot(111, projection="3d")

        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.6, 0.2)
        ax.set_zlim(0, 0.6)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")

        def update(frame):
            ax.cla()
            ax.set_xlim(-0.5, 0.5)
            ax.set_ylim(-0.6, 0.2)
            ax.set_zlim(0, 0.6)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("Z (m)")
            deg = joint_positions[frame]
            q, pos = self.joint_to_pos(deg)
            end_pos = pos[7][:3, 3]
            self.ikpy_chain.plot(q, ax, end_pos)
            ax.set_title(f"Cycling SO-101 Postures\nStep {frame}/{len(joint_positions)-1}", fontsize=11)

        anim = FuncAnimation(fig, update, frames=range(0, len(joint_positions), 4), interval=interval, repeat=True)
        plt.show()
        plt.close()


    def joint_trajectory(self, joint_waypoints, times):
        num_joints = len(joint_waypoints[0])
        time_points = np.arange(0, int(np.sum(times) + 1))

        q_waypoints = np.radians(joint_waypoints)
        positions_rad = np.zeros((num_joints, len(time_points)))

        for j in range(num_joints):
            segment_start_index = 0

            for i in range(len(joint_waypoints) - 1):
                q0 = q_waypoints[i][j]
                qT = q_waypoints[i + 1][j]
                segment_time = times[i]

                # Cubic coefficients (assuming v0 = vT = 0)
                a0 = q0
                a1 = 0
                a2 = 3 * (qT - q0) / (segment_time ** 2)
                a3 = -2 * (qT - q0) / (segment_time ** 3)

                # Generate positions for this segment
                t = np.arange(0, int(segment_time) + 1)
                pos_segment = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3

                # Find the indices for this segment in the overall time array
                segment_end_index = segment_start_index + len(t) - 1
                positions_rad[j, segment_start_index:segment_end_index] = pos_segment[
                    :-1
                ]
                segment_start_index = segment_end_index

            # Add the final point of the last segment
            positions_rad[j, -1] = q_waypoints[-1][j]

        # Convert positions back to degrees for output
        positions_deg = np.degrees(positions_rad)

        joint_positions = []
        for i in time_points:
            q = []
            for j in range(num_joints):
                q.append(positions_deg[j][i])
            joint_positions.append(q)

        return {
            "time_points": time_points,
            "joint_positions": joint_positions,
        }

    STANDARD_POSITIONS = {
        "rest": [0.0, -90.0, 0.0, 90.0, 0.0, 0.0],       # Safe parked / folded pose above base
        "homing": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],        # Middle of servo range (tick 2048)
        "candle": [0.0, 0.0, -90.0, 0.0, 0.0, 0.0],      # Upright vertical pose
        "rotate_left": [90.0, 0.0, 0.0, 90.0, 0.80],     # 90 deg rotation left
        "rotate_right": [-90.0, 0.0, 0.0, 90.0, 0.80],   # 90 deg rotation right
    }

    def get_standard_positions(self) -> dict[str, list[float]]:
        """Return dictionary of standard SO-100/SO-101 reference postures in degrees."""
        return self.STANDARD_POSITIONS.copy()

    def get_standard_cycle_waypoints(self) -> list[list[float]]:
        """Return sequence of waypoints cycling safely through all standard postures:
        rest -> homing -> candle -> homing -> rest.
        """
        cycle = ["rest", "homing", "candle", "homing", "rest"]
        return [self.STANDARD_POSITIONS[k] for k in cycle]

    def plot_trajectory(self, joint_waypoints, times):
        import matplotlib.pyplot as plt

        trajectory = self.joint_trajectory(joint_waypoints, times)
        time_points = trajectory["time_points"]
        joint_positions = trajectory["joint_positions"]

        joint_labels = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]

        for j in range(len(joint_positions[0])):
            positions = [row[j] for row in joint_positions]
            plt.plot(time_points, positions, label=joint_labels[j] if j < len(joint_labels) else f"joint {j}")

        plt.xlabel("Time (steps)")
        plt.ylabel("Joint Angle (degrees)")
        plt.title("Trajectory Cycling Through SO-101 Standard Postures (Degrees)")
        plt.legend(loc="upper right")
        plt.grid()
        plt.show()


if __name__ == "__main__":

    np.set_printoptions(precision=2, suppress=True)

    ikmove = Ikmove()

    std_poses = ikmove.get_standard_positions()
    print("=== SO-101 Standard Reference Postures ===")
    for name, deg in std_poses.items():
        servo = ikmove.joint_to_servo(deg)
        _, pos = ikmove.joint_to_pos(deg)
        xyz = pos[7][:3, 3]
        print(f"[{name.upper():7s}] degrees: {deg}")
        print(f"          servos : {servo.tolist()}")
        print(f"          pos xyz: {[round(float(c), 3) for c in xyz]} m\n")

    # 1. Open 3D GUI window displaying all standard postures:
    print("Opening 3D GUI display for all SO-101 standard postures...")
    ikmove.plot_standard_positions()

    # 2. Cycle through all standard positions with cubic trajectory:
    # rest -> homing -> candle -> zero -> rotated -> candle -> rest
    nwaypoints = ikmove.get_standard_cycle_waypoints()
    ntimes = [150] * (len(nwaypoints) - 1)

    print(f"Plotting trajectory cycling through {len(nwaypoints)} standard waypoints...")
    ikmove.plot_trajectory(nwaypoints, ntimes)



