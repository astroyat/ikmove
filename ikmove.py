#!/usr/bin/env python3

import numpy as np


class Ikmove:
    def __init__(self):
        self.ikpy_chain = None
        self.joint = [0, 0, 0, 0, 0, 0]

    def set_ikpy(self):
        from ikpy.chain import Chain

        self.ikpy_chain = Chain.from_urdf_file(
            urdf_file="arm.urdf",
            active_links_mask=[False, True, True, True, True, True, False, False],
        )

    def joint_to_pos(self, joint):
        if self.ikpy_chain is None:
            self.set_ikpy()

        q_deg = np.zeros(len(joint) + 2)
        q_deg[1] = joint[0]
        q_deg[2] = joint[1]
        q_deg[3] = joint[2]
        q_deg[4] = joint[3] - 90
        q = np.radians(q_deg)

        pos = self.ikpy_chain.forward_kinematics(
            q[: len(self.ikpy_chain.links)], full_kinematics=True
        )
        return q, pos

    def pos_to_joint(self, pos, rot=None, mode=None):
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
        joint[3] = q_deg[4] + 90
        joint[4] = self.joint[4]
        joint[5] = self.joint[5]
        return q, joint

    def joint_to_servo(self, joint):
        q = np.zeros(len(joint))
        q[0] = joint[0]
        q[1] = joint[1]
        q[2] = joint[2]
        q[3] = joint[3] - 90
        q[4] = self.joint[4]
        q[5] = self.joint[5]
        return 2048 + ((q / 180) * 2048)

    def servo_to_joint(self, servo):
        q_deg = (servo - 2048) / 2048 * 180
        joint = np.zeros(len(q_deg))
        joint[0] = q_deg[0]
        joint[1] = q_deg[1]
        joint[2] = q_deg[2]
        joint[3] = q_deg[3] + 90
        joint[4] = self.joint[4]
        joint[5] = self.joint[5]
        return joint

    def plot(self, q, pos):
        import ikpy.utils.plot as plot_utils
        import matplotlib.pyplot as plt

        _, ax = plot_utils.init_3d_figure()
        plt.xlim(-0.5, 0.5)
        plt.ylim(-0.5, 0)
        ax.set_zlim(0, 0.5)

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")

        end_fk = pos[7]
        end_pos = end_fk[:3, 3]
        self.ikpy_chain.plot(q, ax, end_pos)
        plt.show()
        plt.close()

    def plot_deg(self, deg):
        servo = self.joint_to_servo(deg)
        q, pos = self.joint_to_pos(deg)
        end_fk = pos[7]
        end_pos = end_fk[:3, 3]
        end_rot = end_fk[:3, :3]
        print("FK joint: ", deg)
        print("FK servo: ", servo)
        print("FK pos: ", end_pos)
        self.plot(q, pos)

        q, joint = self.pos_to_joint(end_pos, end_rot, "all")
        servo = self.joint_to_servo(joint)
        print("IK joint: ", joint)
        print("IK servo: ", servo)
        print("IK pos: ", end_pos)
        self.plot(q, pos)

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

    def plot_trajectory(self, joint_waypoints, times):
        import matplotlib.pyplot as plt

        trajectory = ikmove.joint_trajectory(joint_waypoints, times)
        time_points = trajectory["time_points"]
        joint_positions = trajectory["joint_positions"]

        for _ in range(len(joint_positions[0])):
            positions = []
            for j in joint_positions:
                positions.append(j)
            plt.plot(time_points, positions)
        plt.xlabel("Time (s)")
        plt.ylabel("Joint (degrees)")
        plt.title("Cubic Polynomial Trajectory for 6-DOF Robot (Degrees)")
        plt.grid()
        plt.show()


if __name__ == "__main__":

    np.set_printoptions(precision=2, suppress=True)

    initial_deg = [0, 0, 0, 90, 0, 0]
    up_deg = [0, 0, -90, 0, 0, 0]
    rotate_left_deg = [90, 0, 0, 90, 0, 0]
    rotate_right_deg = [-90, 0, 0, 90, 0, 0]

    ikmove = Ikmove()

    ikmove.plot_deg(initial_deg)
    ikmove.plot_deg(up_deg)
    ikmove.plot_deg(rotate_left_deg)
    ikmove.plot_deg(rotate_right_deg)

    nwaypoints = [
        initial_deg,
        up_deg,
        rotate_left_deg,
        up_deg,
        rotate_right_deg,
        initial_deg,
    ]
    ntimes = [200, 200, 200, 200, 200, 200, 200, 200, 200]
    ikmove.plot_trajectory(nwaypoints, ntimes)
