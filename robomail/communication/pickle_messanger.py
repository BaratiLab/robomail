import pickle
import socket
from typing import Any, Dict

class PickleMessenger:
    def __init__(self, host_ip:str, port:int, is_host:bool=True) -> None:
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

    def send_object(self, object_name:str, send_object: Any) -> None:
        """
        Sends an object over the socket connection.
        :param object_name: Name of object to send.
        :param send_object: Object to send.
        :return: None
        """
        pickled_dict: bytes = pickle.dumps(obj={"name":object_name, 'data':send_object})
        self.connection.sendall(pickled_dict)
        print("object sent successfully.")

    def receive_object(self) -> tuple[str, Any]:
        """
        Receives an object over the socket connection.
        :return: Tuple of (object name, received object).
        """
        pickled_dict: bytes = self.connection.recv(4096)
        received_object: Dict[str, Any] = pickle.loads(pickled_dict)
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