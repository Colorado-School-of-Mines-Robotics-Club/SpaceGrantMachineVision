from Code.source.hardware.HardwareManager import HardwareManager
from Code.source.hardware.KinematicHardwareInterface import KinematicHardwareInterface
from Code.source.hardware.RobotData import RobotData

try:
    manager = HardwareManager()
except Exception as e:
    print(e)

# at 2.0 m/s should have a max PWM since max_vel is 2.0 -> 4095 for each motor
# at 5 degrees should have a small but equal in magnitude angle in all servos
# should have increasing pwms for the suspensions heights
robotData = RobotData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
interface = KinematicHardwareInterface(robotData=robotData)

print("testing radians_to_pwm function")
for i in range(5):
    val1 = interface.radians_to_pwm(i)
    val2 = interface.radians_to_pwm(i * -1)
    print(f"Radians: {i} -> {val1}, & Radians: {i * -1} -> {val2}")

# baseline commands should be all zeros
print("Baseline all zeros")
print(interface.getCommandPWM())

starting_val = 0.1
print("Wheel velocity testing")
for i in range(25):
    robotData.linear = starting_val * i
    interface.updateFromRobotData(robotData)
    print(interface.getCommandPWM())
robotData.linear = 0.0
interface.updateFromRobotData(robotData)
for i in range(25):
    robotData.linear = starting_val * -1 * i
    interface.updateFromRobotData(robotData)
    print(interface.getCommandPWM())
robotData.linear = 0.0
interface.updateFromRobotData(robotData)

starting_val = 0.1
print("Suspension height testing")
for i in range(100):
    robotData.assignAllHeights(starting_val * i)
    interface.updateFromRobotData(robotData)
    print(interface.getCommandPWM())
robotData.assignAllHeights(0.0)
interface.updateFromRobotData(robotData)
for i in range(100):
    robotData.assignAllHeights(starting_val * -1 * i)
    interface.updateFromRobotData(robotData)
    print(interface.getCommandPWM())
robotData.assignAllHeights(0.0)
interface.updateFromRobotData(robotData)

starting_val = 1
print("Servo/wheel turning")
for i in range(50):
    robotData.angular = starting_val * i
    interface.updateFromRobotData(robotData)
    print(interface.getCommandPWM())
    robotData.angular = starting_val * i * -1
    interface.updateFromRobotData(robotData)
    print(interface.getCommandPWM())
robotData.angular = 0.0
interface.updateFromRobotData(robotData)

print("testing LEDS")
for i in range(2):
    for j in range(2):
        for k in range(2):
            for m in range(2):
                interface.updateLEDs([bool(i), bool(j), bool(k), bool(m)])
                print(interface.getCommandPWM())
