import unittest
import numpy as np
import os
from ikmove import Ikmove

# Set Agg backend for matplotlib during tests to avoid GUI issues
os.environ['MPLBACKEND'] = 'Agg'


class TestIkmove(unittest.TestCase):
    def setUp(self):
        self.ik = Ikmove()

    def test_wrist_roll_affects_fk(self):
        """Test that joint[4] (wrist roll) affects the end-effector rotation and position."""
        # Standard SO-100 postures: upright pose with and without 90 deg roll
        joint1 = [0, 90, 0, 0, 0, 0]
        joint2 = [0, 90, 0, 0, 90, 0]

        _, pos1 = self.ik.joint_to_pos(joint1)
        _, pos2 = self.ik.joint_to_pos(joint2)

        # End effector is at index 7
        p1 = pos1[7][:3, 3]
        p2 = pos2[7][:3, 3]
        r1 = pos1[7][:3, :3]
        r2 = pos2[7][:3, :3]

        self.assertFalse(np.allclose(r1, r2), "Wrist roll (joint[4]) should affect end-effector rotation")

    def test_pos_to_joint_preserves_wrist_roll(self):
        """Test that inverse kinematics correctly retrieves the wrist roll joint."""
        initial_joint = [10, 45, 30, -20, 60, 0]
        _, pos = self.ik.joint_to_pos(initial_joint)
        end_pos = pos[7][:3, 3]
        end_rot = pos[7][:3, :3]

        _, result_joint = self.ik.pos_to_joint(end_pos, end_rot, "all")

        # Check if joint[4] is recovered
        self.assertAlmostEqual(initial_joint[4], result_joint[4], places=1)

    def test_joint_to_servo_and_back(self):
        """Test the conversion between joints and servos includes all 6 joints."""
        initial_joint = [10, 20, 30, 40, 50, 60]
        servo = self.ik.joint_to_servo(initial_joint)

        self.assertEqual(len(servo), 6)

        recovered_joint = self.ik.servo_to_joint(servo)
        # Quantization to integer ticks (4096 steps / 360 deg) gives accuracy within 0.1 deg
        np.testing.assert_array_almost_equal(initial_joint, recovered_joint, decimal=1)

    def test_standard_upright_candle_pose(self):
        """Test standard SO-100 upright pose [0, 90, 0, 0, 0, 0]."""
        candle_joint = [0, 90, 0, 0, 0, 0]
        servos = self.ik.joint_to_servo(candle_joint)
        expected_servos = np.array([2048, 2048, 1024, 1024, 3072, 1024])
        np.testing.assert_array_equal(servos, expected_servos)

        recovered = self.ik.servo_to_joint(servos)
        np.testing.assert_array_almost_equal(candle_joint, recovered, decimal=1)

    def test_standard_zero_degrees_pose(self):
        """Test that [0, 0, 0, 0, 0, 0] in standard matches main_follower.json start_pos."""
        zero_joint = [0, 0, 0, 0, 0, 0]
        servos = self.ik.joint_to_servo(zero_joint)
        expected_start_pos = np.array([2048, 3072, 1024, 1024, 3072, 1024])
        np.testing.assert_array_equal(servos, expected_start_pos)

    def test_plot_trajectory_no_nameerror(self):
        """Test that plot_trajectory doesn't raise NameError when calling joint_trajectory."""
        import matplotlib.pyplot as plt
        original_show = plt.show
        plt.show = lambda: None

        try:
            joint_waypoints = [[0, 90, 0, 0, 0, 0], [10, 45, 10, 0, 10, 0]]
            times = [10]
            self.ik.plot_trajectory(joint_waypoints, times)
        except NameError as e:
            self.fail(f"plot_trajectory raised NameError: {e}")
        except Exception as e:
            self.fail(f"plot_trajectory raised an unexpected exception: {e}")
        finally:
            plt.show = original_show

    def test_cycle_through_so101_standard_positions(self):
        """Test that the full cycle through standard positions computes a valid trajectory."""
        waypoints = self.ik.get_standard_cycle_waypoints()
        self.assertEqual(len(waypoints), 7)
        self.assertEqual(waypoints[0], self.ik.STANDARD_POSITIONS["rest"])
        self.assertEqual(waypoints[-1], self.ik.STANDARD_POSITIONS["rest"])

        times = [50] * (len(waypoints) - 1)
        traj = self.ik.joint_trajectory(waypoints, times)
        self.assertEqual(len(traj["time_points"]), 301)
        self.assertEqual(len(traj["joint_positions"]), 301)

        # Verify start and end postures match rest
        np.testing.assert_allclose(traj["joint_positions"][0], self.ik.STANDARD_POSITIONS["rest"], atol=1e-3)
        np.testing.assert_allclose(traj["joint_positions"][-1], self.ik.STANDARD_POSITIONS["rest"], atol=1e-3)


if __name__ == "__main__":
    unittest.main()


