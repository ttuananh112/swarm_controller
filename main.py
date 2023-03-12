from time import sleep

import src.communication.constants as comm_const
from src.communication.messages import RobotMsg
from src.communication.receiver import Receiver
from src.controller import Mediator

DT = 0.1

if __name__ == "__main__":
    # TODO: unblock mechanism when deadlock
    mediator = Mediator()
    mediator.load_graph(vertices_path="data/vertices.csv", edges_path="data/edges.csv")
    mediator.add_robots_from_file(robots_path="data/robots.csv")

    listener = Receiver(udp_ip=comm_const.SERVER_UDP_IP, udp_port=comm_const.SERVER_UDP_PORT).run()

    while True:
        mediator.update(dt=DT)

        # check msg to update new state from robot
        for msg in listener.get_data():
            if isinstance(msg, RobotMsg):
                # update state and position
                mediator.get_robot_by_id(msg.id).state.set_position(
                    mediator.get_graph().get_node_by_id(msg.pos).position)
                mediator.get_robot_by_id(msg.id).state.set_velocity(msg.vel)
                mediator.get_robot_by_id(msg.id).set_current_position(msg.pos)

        for id_robot, robot in mediator.get_robot_container().items():
            print(robot)
        print("_" * 10)
        sleep(DT)
