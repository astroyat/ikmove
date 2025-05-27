# ikmove
Simple move control routines for <a href="https://github.com/TheRobotStudio/SO-ARM100">SO-ARM100</a> using <a href="https://github.com/Phylliade/ikpy">ikpy</a> for inverse kinematics and cubic polynomial for trajectory planning.

This implementation uses arm.urdf based on SO_5DOF_ARM100_8j_URDF.SLDASM.urdf from SO-ARM100 back in Nov 1, 2024, clone the repo as submodule.
```bash
git submodule update --init
```

The arm needs to be assembled with the goal position of the servos (except for the last two servos) set at 2048 as in the urdf from SO-ARM100, see the urdf below.
```bash
pip install yourdfpy
yourdfpy arm.urdf
```

The goal position at 2048 corresponds to ikpy joint zero degrees that is different from LeRobot joint zero or middle degrees, this implementation converts between LeRobot joint degrees and ikpy joint degrees.

If the arm is not assembled as above, dissemble and reassemble as in the urdf or modify the urdf and the corresponding joint conversion.

Run ikmove.py to see the forward and inverse kinematics, close the windows to switch for different LeRobot calibration joint degrees.

Use main_follower.json if the arm is assembled as above without LeRobot calibration or perform the LeRobot calibration but may result in some differences but should be usable.

Note that the arm moves forward in the negative y direction.
