from robomail.motion import GotoPoseLive
import time
import numpy as np

durration = int(4*np.pi) + 1
hz = 20
t = np.linspace(0, durration, durration*hz)
z = (10*np.sin(t) + 30)/100
x = (10*np.cos(t) + 45)/100
y = (-10 + 20*t/durration)/100

points = np.column_stack([x, y, z])
print(points)
print(points[0])

start_time = time.time()
last_time = -1

controller = GotoPoseLive()
controller.set_goal_translation(points[0])
controller.start()
time.sleep(1)

elapsed_time = 0
while(elapsed_time < durration):
    int_time = int(elapsed_time*hz)
    if int_time > last_time:
        last_time = int_time
        print('t:', int_time, "  point:", points[int_time])
        controller.set_goal_translation(points[int_time])
    elapsed_time = time.time() - start_time

print('stopping')
controller.stop()
        