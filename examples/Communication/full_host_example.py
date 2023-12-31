import robomail.communication as com
import numpy as np
import time
import cv2

# Create a new reciever object
IP:str = '172.26.30.146'
PORT:int = 5000
messenger = com.FullMessenger(IP_Adress=IP, port=PORT, is_host=True)

# Start the reciever
messenger.start()

print('start sleep')
time.sleep(3)
print('end sleep')

# Send an object
messenger.send_object(send_object={'test': np.array([1, 2, 3, 4])}, object_name='test')
time.sleep(3)
messenger.stop()
exit()

print('start send')
messenger.send_object(object_name = "image", send_object=cv2.imread(filename='/home/aigeorge/research/mail-robotics-package/examples/Communication/test.jpg'))
print('end send')
time.sleep(3)
print('stopping')
messenger.stop()


while True:
    message_name, message_object = messenger.wait_for_next_message()
    if message_name == "move_to_command":
        fa.goto_pose(message_object)
    if message_name == "stop":
        break