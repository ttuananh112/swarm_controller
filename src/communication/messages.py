from struct import pack, unpack
from typing import List, Tuple

import src.communication.constants as const


class MessageIsNotReliable(Exception):
    pass


def is_valid_msg(start_value, stop_value) -> bool:
    """
    Check whether message is valid or not
    Args:
        start_value:
        stop_value:
    Returns:
        (bool)
    """
    return start_value == const.START and stop_value == const.STOP


class Message:
    msg_format: str = None
    msg_type: int = None

    def __init__(self):
        self.data_encode = None

    def encode(self):
        """
        Encode message
        Returns:

        """
        return pack(self.msg_format, const.START, self.msg_type, *self.data_encode, const.STOP)

    @classmethod
    def decode_by_format(cls, byte_arr):
        """
        Decode message by msg_format
        Args:
            byte_arr:

        Returns:

        """
        raise NotImplementedError

    @staticmethod
    def decode(byte_arr):
        """
        Decode msg base on its msg_type
        Args:
            byte_arr:

        Returns:
            Message object
        """
        if len(byte_arr) < 3:
            raise MessageIsNotReliable("Len of byte_arr must >= 3")
        start = byte_arr[0]
        msg_type = byte_arr[1]
        stop = byte_arr[-1]

        if not is_valid_msg(start, stop):
            raise MessageIsNotReliable("Message is not reliable")

        if msg_type not in MAPPING:
            raise KeyError(f"Not valid msg_type, got {msg_type}")

        return MAPPING[msg_type].decode_by_format(byte_arr)

    def __repr__(self):
        return f"{self.__class__.__name__}:{str(self.__dict__)}"


class PathMsg(Message):
    msg_format: str = const.PATH_MSG_FORMAT
    msg_type: int = const.PATH_MSG_TYPE

    def __init__(self, id: int, path: List[int]):
        """

        Args:
            id (int): robot index
            path (List[int]): path that robot should follow
        """
        super().__init__()
        self.id = id
        self.path = path

        self.data_encode: Tuple = (self.id, *self.path)

    @classmethod
    def decode_by_format(cls, byte_arr):
        start, msg_type, *data, stop = unpack(cls.msg_format, byte_arr)
        id, *path = data
        return PathMsg(id=id, path=path)


class RobotMsg(Message):
    msg_format: str = const.ROBOT_MSG_FORMAT
    msg_type: int = const.ROBOT_MSG_TYPE

    def __init__(self, id: int, pos: int, vel: float):
        """

        Args:
            id (int): robot index
            pos (int): node index
            vel (float): velocity of robot [m/s]
        """
        super().__init__()
        self.id = id
        self.pos = pos
        self.vel = vel

        self.data_encode: Tuple = (self.id, self.pos, self.vel)

    @classmethod
    def decode_by_format(cls, byte_arr):
        start, msg_type, *data, stop = unpack(cls.msg_format, byte_arr)
        id, pos, vel = data
        return RobotMsg(id=id, pos=pos, vel=vel)


class StatusMsg(Message):
    msg_format: str = const.STATUS_MSG_FORMAT
    msg_type: int = const.STATUS_MSG_TYPE

    def __init__(self, is_success: bool):
        """

        Args:
            is_success (bool): success flag
        """
        super().__init__()
        self.is_success = is_success
        self.data_encode = (is_success,)

    @classmethod
    def decode_by_format(cls, byte_arr):
        start, msg_type, is_success, stop = unpack(cls.msg_format, byte_arr)
        return StatusMsg(is_success=is_success)


MAPPING = {
    const.STATUS_MSG_TYPE: StatusMsg,
    const.ROBOT_MSG_TYPE: RobotMsg,
    const.PATH_MSG_TYPE: PathMsg
}
