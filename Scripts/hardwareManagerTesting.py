import sys
import time

from Code.source.hardware import HardwareManager, KinematicHardwareInterface, RobotData


hardware = HardwareManager(feedback=False)
print("created hardware manager")
hardware = hardware.start_threads()
robotData = RobotData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
interface = KinematicHardwareInterface(robotData=robotData)

stepSize = 0.1
val = -3.0
counter = 0
while val + stepSize * counter <= 3.0:
    robotData.linear = val + stepSize * counter
    interface.updateFromRobotData(robotData=robotData)
    commands = [(interface.getCommandPWM(), 0.1) for i in range(20)]
    hardware.update_pwm_targets(commands)
    counter += 1

time.sleep(5)
hardware.stop()
