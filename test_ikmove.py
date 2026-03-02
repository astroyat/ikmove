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
        joint1 = [0, 0, 0, 90, 0, 0]
        joint2 = [0, 0, 0, 90, 90, 0]

        _, pos1 = self.ik.joint_to_pos(joint1)
        _, pos2 = self.ik.joint_to_pos(joint2)

        # End effector is at index 7
        p1 = pos1[7][:3, 3]
        p2 = pos2[7][:3, 3]
        r1 = pos1[7][:3, :3]
        r2 = pos2[7][:3, :3]

        # In the original bug, both position and rotation were identical
        self.assertFalse(np.allclose(r1, r2), "Wrist roll (joint[4]) should affect end-effector rotation")

        # NOTE: Wrist roll might not affect the origin of the last link if the link is aligned with the roll axis.
        # But it should definitely affect the rotation.

    def test_pos_to_joint_preserves_wrist_roll(self):
        """Test that inverse kinematics correctly retrieves the wrist roll joint."""
        initial_joint = [10, -20, 30, 45, 60, 0]
        _, pos = self.ik.joint_to_pos(initial_joint)
        end_pos = pos[7][:3, 3]
        end_rot = pos[7][:3, :3]

        _, result_joint = self.ik.pos_to_joint(end_pos, end_rot, "all")

        # Check if joint[4] is recovered (approximately, as it's IK)
        self.assertAlmostEqual(initial_joint[4], result_joint[4], places=1)

    def test_joint_to_servo_and_back(self):
        """Test the conversion between joints and servos includes all 6 joints."""
        # Test values including wrist roll and gripper
        initial_joint = [10, 20, 30, 40, 50, 60]
        servo = self.ik.joint_to_servo(initial_joint)

        # Should have 6 elements
        self.assertEqual(len(servo), 6)

        # Convert back
        recovered_joint = self.ik.servo_to_joint(servo)

        # Should be consistent
        np.testing.assert_array_almost_equal(initial_joint, recovered_joint)

    def test_plot_trajectory_no_nameerror(self):
        """Test that plot_trajectory doesn't raise NameError when calling joint_trajectory."""
        import matplotlib.pyplot as plt
        # Mock plt.show to do nothing
        original_show = plt.show
        plt.show = lambda: None

        try:
            joint_waypoints = [[0,0,0,90,0,0], [10,10,10,90,10,0]]
            times = [10]
            self.ik.plot_trajectory(joint_waypoints, times)
        except NameError as e:
            self.fail(f"plot_trajectory raised NameError: {e}")
        except Exception as e:
            self.fail(f"plot_trajectory raised an unexpected exception: {e}")
        finally:
            plt.show = original_show

if __name__ == "__main__":
    unittest.main()
