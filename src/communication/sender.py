import socket

import src.communication.constants as const
from src.communication.messages import RobotMsg, StatusMsg, PathMsg


class Sender:
    def __init__(
            self,
            udp_ip: str,
            udp_port: int
    ):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # create UDP socket

    def send_message(self, message):
        self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))  # send message to server


if __name__ == "__main__":
    Sender(udp_ip=const.SERVER_UDP_IP, udp_port=const.SERVER_UDP_PORT).send_message(RobotMsg(id=1, pos=33, vel=1))
    Sender(udp_ip=const.SERVER_UDP_IP, udp_port=const.SERVER_UDP_PORT).send_message(StatusMsg(True))
    Sender(udp_ip=const.SERVER_UDP_IP, udp_port=const.SERVER_UDP_PORT).send_message(
        PathMsg(id=2, path=[i for i in range(10)]))
