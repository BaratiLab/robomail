from logging import warning
from .pickle_messanger import PickleMessenger
from typing import Any, Dict, Tuple, List
import threading 

class FullMessenger():
    """
    Class to send and receive objects over a socket connection. Stores all sent and recieved messages.
    """
    def __init__(self, IP_Adress: str, port: int, is_host: bool) -> None:
        """
        Class to send and receive objects over a socket connection. Stores all sent and recieved messages.
        :param host_ip: IP address of host.
        :param port: Port to connect to.
        :param is_host: Whether this instance is the host or not.
        :return: None
        """
        self.messenger = PickleMessenger(host_ip=IP_Adress, port=port, is_host=is_host)
        self.recieved_messgaes: Dict[str, Any] = {}
        self.sent_messages: Dict[str, List[Any]] = {}
        self.running: bool = False
        self.last_message: Tuple[str, Any] = ("None", None)
        self.num_messages_recieved: int = 0

    def listen(self) -> None:
        """
        Ran in a seperate thread. Continuously listens for new messages.
        :return: None
        """
        self.messenger.start()
        self.running = True
        while self.running:
            new_message: Tuple[str, Any] = self.messenger.receive_object()
            if self.messenger.connection_open == False:
                self.running = False
                break
            if new_message[0] not in self.recieved_messgaes.keys():
                self.recieved_messgaes[new_message[0]] = []
            self.recieved_messgaes[new_message[0]].append(new_message[1])
            self.last_message = new_message
            self.num_messages_recieved += 1
    
    def start(self) -> None:
        """
        Starts the socket connection and the listening thread.
        :return: None
        """
        self.reciever_thread = threading.Thread(target=self.listen)
        self.reciever_thread.start()

    def stop(self) -> None:
        """
        Stops the socket connection and the listening thread.
        :return: None
        """
        self.running = False
        self.messenger.close()
        self.reciever_thread.join()

    def send_object(self, send_object: Any, object_name: str) -> None:
        """
        Sends an object over the socket connection.
        :param send_object: Object to send.
        :param object_name: Name of object to send.
        :return: None
        """
        self.messenger.send_object(send_object=send_object, object_name=object_name)
        if object_name not in self.sent_messages.keys():
            self.sent_messages[object_name] = []
        self.sent_messages[object_name].append(send_object)

    def recieve_object(self, object_name: str, wait: bool = False) -> Any:
        """
        Returns the last recieved object with the given name. If wait is true, waits until a new message is recieved before returning.
        :param object_name: Name of object to return.
        :param wait: Whether to wait for a new message to be recieved before returning.
        :return: Last recieved object with the given name.
        """
        if object_name not in self.recieved_messgaes.keys():
            if wait:
                self.recieved_messgaes[object_name] = []
            else:
                warning(f"Object {object_name} has not been recieved yet.")
                return None
        if wait:
            previous_num_messages_recieved: int = len(self.recieved_messgaes[object_name])
            while len(self.recieved_messgaes[object_name]) == previous_num_messages_recieved:
                pass
            return self.recieved_messgaes[object_name][-1]
        else:
            return self.recieved_messgaes[object_name][-1]
        
    def wait_for_next_message(self) -> Tuple[str, Any]:
        """
        Waits until a new message and returns it.
        :return: Tuple of (object name, object).
        """
        previous_num_messages_recieved: int = self.num_messages_recieved
        while self.num_messages_recieved == previous_num_messages_recieved:
            pass
        return self.last_message
    
            



