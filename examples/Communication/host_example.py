import robomail.communication as com
import numpy as np
import time
import cv2

# Create a new reciever object
IP:str = '172.26.30.146'
PORT:int = 5000
messenger = com.PickleMessenger(host_ip=IP, port=PORT, is_host=True)

# Start the reciever
messenger.start()

print('start sleep')
time.sleep(5)
print('end sleep')

# Send an object
messenger.send_object(send_object={'test': np.array([1, 2, 3, 4])}, object_name='test')
time.sleep(5)

# send an image
messenger.send_object(object_name = "image", send_object=cv2.imread(filename='test.png'))

