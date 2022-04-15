from .HardwareManager import HardwareManager
from typing import Tuple, Union, Dict


# function to create object on other side of the payloadManager
def createHardwareManager(args: Tuple) -> Tuple[Union[HardwareManager, None]]:
    return HardwareManager().start_threads(),


def PThardwareCommand(args: Tuple) -> Union[Dict, None]:
    queue = args[0]
    hardware: HardwareManager = args[1]
    feedback = args[2]

    # uses getInputs and then acquires the last command to ensure newest data is sent
    commands = queue.getInputs()
    if len(commands) != 0:
        command = commands[len(commands) - 1]
        hardware.update_pwm_targets(command)

    if feedback:
        return {'motors': hardware.get_curr_motors(), 'servos': hardware.get_curr_servos(),
                'past_accel': hardware.get_past_accel(), 'curr_accel': hardware.get_curr_accel(),
                'past_gyro': hardware.get_past_gyro(), 'curr_gyro': hardware.get_curr_gyro()}
    return None
