from frankapy import FrankaArm
from robomail.motion import GotoJointsLive
import time
import numpy as np

fa = FrankaArm()

fa.reset_joints()
pose = fa.get_pose()
pose.translation = np.array([.6, 0, .4])
fa.goto_pose(pose)
# time.sleep(10)
joints = fa.get_joints()
print('start joints:', joints)

durration = 10
hz = 10
joint_4 = np.linspace(-np.pi/6, np.pi/6, int(hz*durration)) + joints[4]
# print('joint_4:', joint_4)

goal_joints = np.copy(joints)
goal_joints[4] = joint_4[0]
controller = GotoJointsLive(ignore_virtual_walls=True)
print('controller init')
controller.set_goal_joints(goal_joints)
print('controller set')
print('controller start')
controller.start()
time.sleep(5)
print('start motion')

start_time = time.time()
last_time = -1

elapsed_time = 0
while(elapsed_time < durration):
    int_time = int(elapsed_time*hz)
    if int_time > last_time:
        last_time = int_time
        goal_joints[4] = joint_4[int_time]
        print('t:', int_time, "  joints:", goal_joints)
        controller.set_goal_joints(goal_joints)
    elapsed_time = time.time() - start_time

print('finished with updates')
time.sleep(2)
print('stopping')
controller.stop()