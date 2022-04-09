import pickle
from threading import Thread
import time
import sys
from dataclasses import dataclass
import keyboard
import socket

from .logger import Logger

@dataclass
class RobotData:
    linear: float
    angular: float
    fl_height: float
    fr_height: float
    bl_height: float
    br_height: float

    def incrementAllHeights(self, delta):
        self.fl_height += delta
        self.fr_height += delta
        self.bl_height += delta
        self.br_height += delta

    def roundData(self):
        self.linear = round(self.linear, 1)
        self.angular = round(self.angular, 1)
        self.fl_height = round(self.fl_height, 1)
        self.fr_height = round(self.fr_height, 1)
        self.bl_height = round(self.bl_height, 1)
        self.br_height = round(self.br_height, 1)
    
    def from_list(self, data):
        self.linear = data[0]
        self.angular = data[1]
        self.fl_height = data[2]
        self.fr_height = data[3]
        self.bl_height = data[4]
        self.br_height = data[5]



vel_data = RobotData(linear=0.0, angular=0.0, fl_height=0.0, fr_height=0.0, bl_height=0.0, br_height=0.0)
global_shutdown = False

def incomingDataLoop(tcp_port: int = 9500):
    server_address = ("localhost", tcp_port)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server_socket.settimeout(1.0)
    server_socket.bind(server_address)
    server_socket.listen(5)

    client_socket = None

    while client_socket is None:
        try:
            client_socket, client_address = server_socket.accept() # wait for client
            Logger.log("Controller connected")
        except socket.timeout:
            Logger.log("Waiting for controller...")
            time.sleep(1.0)
            pass

    while not global_shutdown:
        incoming_data = client_socket.recv(4096)
        if not incoming_data:
            Logger.log("Connection closed.")
            break
        final_data = pickle.loads(incoming_data)
        vel_data.from_list(final_data)
    return

def remoteControl(hz: float = 60.0) -> None:

    while not global_shutdown:
        incomingDataLoop()
        time.sleep(1.0)
