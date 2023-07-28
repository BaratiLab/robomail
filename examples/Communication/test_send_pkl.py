import pickle
import socket

def send_dictionary(dictionary, host, port):
    # Pickle the dictionary
    pickled_dict = pickle.dumps(dictionary)

    # Create a socket connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # Connect to the receiver
        s.connect((host, port))

        # Send the pickled dictionary
        s.sendall(pickled_dict)

        # Receive response from the receiver
        response = s.recv(4096)

    print("Dictionary sent successfully.")
    print("Response received:", response.decode())


# Define the dictionary to be sent
my_dictionary = {"name": "John", "age": 30, "city": "New York"}

# Specify the receiver's IP address and port
receiver_host = '127.0.0.1'  # Replace with the receiver's IP address
receiver_port = 12345  # Replace with the receiver's port

# Call the function to send the dictionary
send_dictionary(my_dictionary, receiver_host, receiver_port)
