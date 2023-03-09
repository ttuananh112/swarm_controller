import socket

import src.communication.constants as const

MESSAGE = "Hello, World!"


class Sender:
    def __init__(
            self,
            udp_ip: str = const.UDP_IP,
            udp_port: int = const.UDP_PORT
    ):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # create UDP socket

    def send_message(self, message):
        self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))  # send message to server


if __name__ == "__main__":
    Sender().send_message(MESSAGE)
