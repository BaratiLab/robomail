import robomail.communication as com
import numpy as np
import time

# Create a new reciever object
IP:str = '127.0.0.1'
PORT:int = 5000
messenger = com.PickleMessenger(host_ip='IP', port=PORT, is_host=True)

# Start the reciever
messenger.start()

time.sleep(secs=1)

# Send an object
messenger.send_object(send_object={'test': np.array([1, 2, 3, 4])}, object_name='test')

