from robomail.motion import GotoPoseGripperLive
from frankapy import FrankaArm
import time
import numpy as np
from typing import List, Tuple, Dict, Any, Union

print('start program')

trajectory:List[Tuple[float, np.ndarray, float]] = []

for t in np.arange (0, 5, 0.25):
    trajectory.append((t, np.array([0.5, 0, 0.525-0.1*t]), np.random.uniform(0.07, 0.08)))

for t in np.arange(5, 8, 0.25):
    trajectory.append((t, np.array([0.5, 0, 0.025]), np.random.uniform(0, 0.02)))

for t in np.arange(8, 13, 0.25):
    trajectory.append((t, np.array([0.5, 0, 0.025+0.1*(t-8)]), np.random.uniform(0, 0.02)))

for t in np.arange(13, 18, 0.25):
    trajectory.append((t, np.array([0.5, 0, 0.525]), np.random.uniform(0.07, 0.08)))

fa = FrankaArm()
fa.reset_joints()

fa.open_gripper()
time.sleep(3)

controller = GotoPoseGripperLive()
controller.set_goal_translation(np.array([0.5, 0, 0.525]))
controller.start()
time.sleep(3)
for t, pos, gripper in trajectory:
    controller.set_goal_translation(pos)
    controller.set_gripper_width(gripper)
    time.sleep(0.25)

controller.stop()

print('program terminated')