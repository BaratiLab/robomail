from dictionary_messenger import DictionaryMessenger

# Usage Example
receiver_host = '172.26.30.146'  # Replace with the receiver's IP address
receiver_port = 12345  # Replace with the receiver's port

messenger = DictionaryMessenger(receiver_host, receiver_port)

# If server:
messenger.start()

# If client:
# messenger.connect()

while True:
    # Send a dictionary
    my_dictionary = {"name": "John", "age": 30, "city": "New York"}
    messenger.send_dictionary(my_dictionary)

    # Receive a dictionary
    received_dict = messenger.receive_dictionary()

    # Modify the received dictionary
    received_dict["age"] += 1

    # Send the modified dictionary back
    messenger.send_dictionary(received_dict)

    # Break the loop if a certain condition is met
    if received_dict["age"] >= 40:
        break

messenger.close()