import pickle
import socket

def receive_dictionary(host, port):
    # Create a socket connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # Bind the socket to a specific host and port
        s.bind((host, port))

        # Listen for incoming connections
        s.listen(1)

        # Accept the connection
        conn, addr = s.accept()

        # Receive the pickled dictionary
        pickled_dict = conn.recv(4096)

        # Unpickle the dictionary
        dictionary = pickle.loads(pickled_dict)

        # Process the dictionary
        print("Received Dictionary:")
        print(dictionary)

        # Send a response back to the sender
        response = "Dictionary received successfully.".encode()
        conn.sendall(response)


# Specify the receiver's IP address and port
receiver_host = '127.0.0.1'  # Replace with the receiver's IP address
receiver_port = 12345  # Replace with the receiver's port

# Call the function to receive the dictionary
receive_dictionary(receiver_host, receiver_port)