import socket
from threading import Thread

import src.communication.constants as const
from src.common import logger


# singleton

class Receiver:
    _instance = None

    def __new__(cls):
        # Check if an instance already exists
        if cls._instance is None:
            # Call the superclass __new__ method to create a new instance
            cls._instance = super(Receiver, cls).__new__(cls)
        # Return the instance
        return cls._instance

    def __init__(
            self,
            udp_ip: str = const.UDP_IP,
            udp_port: int = const.UDP_PORT
    ):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # create UDP socket
        self.sock.bind((self.udp_ip, self.udp_port))  # bind to address and port
        self.listener = Thread(target=self.start_listening)
        self.container = []

    def run(self):
        self.listener.start()
        return self

    def stop(self):
        logger.info("Stop receiving")
        self.listener.join()

    def start_listening(self):
        while True:
            data, addr = self.sock.recvfrom(const.BUFFER_SIZE)  # receive up to 1024 bytes of data
            self.container.append(data.decode())

    def get_data(self):
        if len(self.container) > 0:
            return self.container.pop(0)
        return None


if __name__ == "__main__":
    recv = Receiver().run()
    while True:
        try:
            data = recv.get_data()
            if data:
                print(data)
        except KeyboardInterrupt:
            recv.stop()
            break
