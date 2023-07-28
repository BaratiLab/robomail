import pickle
import socket


class DictionaryMessenger:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = None
        self.connection = None

    def start(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        print(f"Listening for incoming connections on {self.host}:{self.port}...")

        self.connection, _ = self.socket.accept()
        print("Connection established.")

    def connect(self):
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection.connect((self.host, self.port))
        print(f"Connected to {self.host}:{self.port}.")

    def send_dictionary(self, dictionary):
        pickled_dict = pickle.dumps(dictionary)
        self.connection.sendall(pickled_dict)
        print("Dictionary sent successfully.")

    def receive_dictionary(self):
        pickled_dict = self.connection.recv(4096)
        dictionary = pickle.loads(pickled_dict)
        print("Received Dictionary:")
        print(dictionary)
        return dictionary

    def close(self):
        if self.connection:
            self.connection.close()
        if self.socket:
            self.socket.close()
