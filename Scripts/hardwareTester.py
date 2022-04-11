from Code.source.hardware.HardwareManager import HardwareManager
from Code.source.hardware.KinematicHardwareInterface import KinematicHardwareInterface
from Code.source.hardware.RobotData import RobotData

try:
    manager = HardwareManager()
except Exception as e:
    print(e)

robotData = RobotData(0, 0, 0, 0, 0, 0)
interface = KinematicHardwareInterface()


for j in range(10):
    robotData.linear = j
    robotData.angular = j
    interface.updateFromRobotData(robotData)
    print(interface.getCommandPWM())
