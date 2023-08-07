import robomail.communication as com
import numpy as np
# import time
from typing import Any, Tuple

# Create a new reciever object
IP:str = '172.26.30.146'
PORT:int = 5000
messenger = com.FullMessenger(IP_Adress=IP, port=PORT, is_host=False)

# Start the reciever
messenger.start()

# Send an object
received_message: Tuple[str, Any] = messenger.wait_for_next_message()

print(f"Recieved object {received_message[0]} with value {received_message[1]}.")

