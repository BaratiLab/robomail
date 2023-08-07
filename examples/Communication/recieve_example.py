import robomail.communication as com
from typing import Any, Dict

# Create a new reciever object
IP:str = '127.0.0.1'
PORT:int = 5000
messenger = com.PickleMessenger(host_ip='IP', port=PORT, is_host=False)

# Start the reciever
messenger.start()
recieved_object, object_name = messenger.recieve_object(object_name='test')

print(f"Recieved object {object_name} with value {recieved_object}.")

