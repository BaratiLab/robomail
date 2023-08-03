from frankapy import FrankaArm
from robomail.motion import GotoJointsLive
import time
import numpy as np

fa = FrankaArm()

# Reset the arm to a good position 
fa.reset_joints()
pose = fa.get_pose()
pose.translation = np.array([.6, 0, .4])
fa.goto_pose(pose)

# Read the intial joints
joints = fa.get_joints()
print('start joints:', joints)

# The code will move the forth joint from -30 deg from start to +30 deg from start, over *durration* seconds at *hz* hz.
durration = 10
hz = 10
joint_4 = np.linspace(-np.pi/6, np.pi/6, int(hz*durration)) + joints[4]

# Set goal joints forth joint to the first value for joint 4.
goal_joints = np.copy(joints)
goal_joints[4] = joint_4[0]

# initalize the joint controller
controller = GotoJointsLive(ignore_virtual_walls=True)
print('controller init')

# Set the goal joint to the first value
controller.set_goal_joints(goal_joints)
print('controller set')

# Start the controller
controller.start()
print('controller start')
time.sleep(5) # wait while the robot moves to the first location

# start moving through the points 
print('start motion')
start_time = time.time()
last_time = -1

elapsed_time = 0
# update the location every 1/hz seconds. 
while(elapsed_time < durration):
    int_time = int(elapsed_time*hz) 
    if int_time > last_time:
        last_time = int_time
        goal_joints[4] = joint_4[int_time] # update the new joint location
        print('t:', int_time, "  joints:", goal_joints) 
        controller.set_goal_joints(goal_joints) # tell the controller to move the joints to the new values.
    elapsed_time = time.time() - start_time

print('finished with updates')
time.sleep(2)
print('stopping')
controller.stop() # stop the controller