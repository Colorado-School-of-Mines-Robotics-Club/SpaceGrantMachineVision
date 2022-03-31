from .HardwareManager import HardwareManager
from typing import Tuple


# function to create object on other side of the payloadManager
def createHardwareManager(args: Tuple) -> HardwareManager:
    return HardwareManager()


def PThardwareCommand(args: Tuple):
    queue = args[0]
    hardware: HardwareManager = args[1]

    commands = queue.getInput()

    for command in commands:
        # TODO
        # do something with the commands
        pass

    return {'motors': hardware.get_curr_motors(), 'servos': hardware.get_curr_servos(),
            'past_accel': hardware.get_past_accel(), 'curr_accel': hardware.get_curr_accel(),
            'past_gyro': hardware.get_past_gyro(), 'curr_gyro': hardware.get_curr_gyro()}
