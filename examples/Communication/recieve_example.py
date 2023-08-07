import robomail.communication as com
from typing import Any
import numpy

# Create a new reciever object
IP:str = '127.0.0.1'
PORT:int = 5000
messenger = com.PickleMessenger(host_ip=IP, port=PORT, is_host=False)

# Start the reciever
messenger.start()
recieved_object: Any
object_name: str
object_name, recieved_object = messenger.receive_object()

print(f"Recieved object {object_name} with value {recieved_object}.")

