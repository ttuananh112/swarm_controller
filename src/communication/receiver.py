import socket
from threading import Thread

import src.communication.constants as const
from src.common import logger
from src.communication.messages import Message


class Receiver:
    # singleton
    _instance = None

    def __new__(cls, *args, **kwargs):
        # Check if an instance already exists
        if cls._instance is None:
            # Call the superclass __new__ method to create a new instance
            cls._instance = super(Receiver, cls).__new__(cls)
        # Return the instance
        return cls._instance

    def __init__(
            self,
            udp_ip: str = const.SERVER_UDP_IP,
            udp_port: int = const.SERVER_UDP_PORT,
    ):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # create UDP socket
        self.sock.bind((self.udp_ip, self.udp_port))  # bind to address and port

        self.container = []
        self.listener = Thread(target=self.start_listening)

    def run(self):
        self.listener.start()
        return self

    def stop(self):
        logger.info("Stop receiving")
        self.listener.join()

    def start_listening(self):
        while True:
            data, addr = self.sock.recvfrom(const.BUFFER_SIZE)  # receive up to 1024 bytes of data
            self.container.append(Message.decode(data))

    def get_data(self):
        if len(self.container) > 0:
            return self.container.pop(0)
        return None


if __name__ == "__main__":
    recv = Receiver().run()
    while True:
        try:
            robot_data = recv.get_data()
            if robot_data:
                print(robot_data)

        except KeyboardInterrupt:
            recv.stop()
            break
