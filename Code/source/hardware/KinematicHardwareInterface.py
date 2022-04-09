from typing import Dict, Callable

try:
    from .RobotData import RobotData
    from source.utilities.Config import Config
    from source.kinematics import KinematicModel
except ModuleNotFoundError as e:
    try:
        from .RobotData import RobotData
        from Code.source.utilities.Config import Config
        from Code.source.kinematics import KinematicModel
    except ModuleNotFoundError:
        raise e


class KinematicHardwareInterface:
    def __init__(self):
        self.kinematicModel = KinematicModel()
        self.RobotData = RobotData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


# pwms
#   'linear': pwm to assign all motors
#   'angular': pwm to assign all wheel servos
#   'height': pwm to assign all suspension servos
def PWM_to_RobotData(robot: RobotData, pwms: Dict, pwm_to_vel: Callable, pwm_to_theta: Callable):
    robot.linear = pwm_to_vel(pwms['linear'])
    robot.angular = pwm_to_theta(pwms['angular'])

