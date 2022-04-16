import pickle
from threading import Thread
import time
import sys
from dataclasses import dataclass
import keyboard
import socket
from typing import Union
import math

try:
    from .hardware import RobotData, HardwareManager, KinematicHardwareInterface
except ModuleNotFoundError as e:
    try:
        from Code.source.hardware import RobotData, HardwareManager, KinematicHardwareInterface
    except ModuleNotFoundError:
        raise e

try:
    import RPi.GPIO as GPIO
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    from smbus2 import *
    from digi.xbee.devices import XBeeDevice
    import board
    from board import SCL, SDA
    import busio
    from adafruit_motor import servo
    from adafruit_pca9685 import PCA9685
    from adafruit_mpu6050 import MPU6050
except ModuleNotFoundError:
    pass
except ImportError:
    pass
except NotImplementedError:
    pass

from .logger import Logger


vel_data = RobotData(linear=0.0, angular=0.0, fl_height=0.0, fr_height=0.0, bl_height=0.0, br_height=0.0)
global_shutdown = False

hardware: Union[HardwareManager, None] = None
interface: Union[KinematicHardwareInterface, None] = None


def incomingDataLoop(tcp_port: int = 9500):
    server_address = ('', tcp_port)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server_socket.settimeout(1.0)
    server_socket.bind(server_address)
    server_socket.listen(5)

    client_socket = None

    while client_socket is None:
        try:
            client_socket, client_address = server_socket.accept()  # wait for client
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
        try:
            final_data = pickle.loads(incoming_data)
            global vel_data
            vel_data.from_list(final_data)
        except:
            Logger.log("Net Exception")
            continue

        # Update the robot
        global interface
        global hardware
        # print(f"Received command: {vel_data}")
        interface.updateFromRobotData(robotData=vel_data)
        commandTargets = interface.getCommandTargets()
        hardware.update_targets([(commandTargets, 2)])


    return




def remoteControl(hz: float = 60.0) -> None:

    # Initialize robot here
    global hardware
    try:
        hardware = HardwareManager().start_threads()
    except NameError:
        hardware = None
    time.sleep(0.5)
    global interface
    global vel_data
    interface = KinematicHardwareInterface(robotData = vel_data)

    while not global_shutdown:
        incomingDataLoop()  # This function loops until the connection is closed.
        time.sleep(1.0)
    
    # Shutdown robot here
    if hardware is not None:
        hardware.shutdown()
