import pickle
import socket
from typing import Any, Dict

class PickleMessenger:
    def __init__(self, host_ip:str, port:int, is_host:bool=True, packet_size:int = 4096) -> None:
        """
        Class to send and receive objects over a socket connection.
        :param host_ip: IP address of host.
        :param port: Port to connect to.
        :param is_host: Whether this instance is the host or not.
        :return: None
        """
        self.host:str = host_ip
        self.port: int = port
        self.is_host: bool = is_host
        self.connection_open: bool = False
        self.packet_size: int = packet_size

    def start(self) -> None:
        """
        Starts the socket connection.
        :return: None
        """
        if self.is_host:
            self.host_server()
            print('hositing socket')
        else:
            self.connect()
            print('connecting to socket')
        
        self.connection_open = True

    def host_server(self) -> None:
        """
        Hosts the socket connection.
        :return: None
        """
        if not self.is_host:
            raise Exception("Cannot start PickleMessenger if not host.")
        self.socket: socket.socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        print(f"Listening for incoming connections on {self.host}:{self.port}...")
        self.connection: socket.socket
        self.connection, _ = self.socket.accept()
        print("Connection established.")

    def connect(self) -> None:
        """"
        Connects to the socket connection.
        :return: None
        """
        if self.is_host:
            raise Exception("Cannot connect PickleMessenger if host.")
        self.connection: socket.socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
        self.connection.connect((self.host, self.port))
        print(f"Connected to {self.host}:{self.port}.")

    def send_object(self, object_name: str, send_object: Any) -> None:
        """
        Sends an object over the socket connection.
        :param object_name: Name of object to send.
        :param send_object: Object to send.
        :return: None
        """
        pickled_dict: bytes = pickle.dumps({"name": object_name, 'data': send_object})

        # Send the size of the pickled data first
        size_data = len(pickled_dict).to_bytes(8, byteorder='big')
        print(f"Sending object of size {len(pickled_dict)}.")
        self.connection.sendall(size_data)

        # Send the pickled data in chunks
        offset: int = 0
        while offset < len(pickled_dict):
            self.connection.sendall(pickled_dict[offset:offset + self.packet_size])
            offset += self.packet_size

        print("Object sent successfully.")

    def receive_object(self) -> tuple[str, Any]:
        """
        Receives an object over the socket connection.
        :return: Tuple of (object name, received object).
        """
        # Receive the size of the pickled data
        size_data = self.connection.recv(8)
        if not size_data:
            print("Connection closed by the other end.")
            self.connection_open = False
            return ("None", None)
        size = int.from_bytes(size_data, byteorder='big')

        # Receive the pickled data in chunks and reconstruct it
        received_data = b""
        while size > 0:
            data = self.connection.recv(min(self.packet_size, size))
            if not data:
                break
            received_data += data
            size -= len(data)

        if len(received_data) == 0:
            print("Connection closed by the other end.")
            self.connection_open = False
            return ("None", None)

        received_object: Dict[str, Any] = pickle.loads(received_data)
        data_object: Any = received_object['data']
        name: str = received_object['name']
        print("Received object:")
        print(received_object)
        return name, data_object

    def close(self) -> None:
        """
        Closes the socket connection.
        :return: None
        """
        if self.connection:
            self.connection.close()
        if self.socket:
            self.socket.close()