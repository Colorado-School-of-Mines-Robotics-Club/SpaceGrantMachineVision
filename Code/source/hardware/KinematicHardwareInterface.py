from typing import Dict, Callable, List, Union

try:
    from .RobotData import RobotData
    from source.utilities.Config import Config
    from source.kinematics.KinematicModel import KinematicModel
except ModuleNotFoundError as e:
    try:
        from .RobotData import RobotData
        from Code.source.utilities.Config import Config
        from Code.source.kinematics.KinematicModel import KinematicModel
    except ModuleNotFoundError:
        raise e


class KinematicHardwareInterface:
    def __init__(self):
        self.kinematicModel = KinematicModel()
        self.robotData = RobotData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    @staticmethod
    def ms_to_pwm(ms: float) -> int:
        # TODO
        return 0

    @staticmethod
    def radians_to_pwm(rad: float) -> int:
        # TODO
        return 0

    @staticmethod
    def pwm_to_ms(pwm: int) -> float:
        # TODO
        return 0

    @staticmethod
    def pwm_to_radians(pwm: int) -> float:
        # TODO
        return 0

    # Will take robotData parameters and run them through the kinematic model
    # will return m/s for motors and angles in radians for servos
    def updateFromRobotData(self, robotData: Union[RobotData, None] = None) -> List[Union[float, int]]:
        if robotData is not None:
            self.robotData = robotData
        self.kinematicModel.update(wheelVelocities=[self.robotData.linear for i in range(4)],
                                   swerveAngles=[self.robotData.angular for i in range(4)],
                                   suspensionHeightTargets=[self.robotData.fl_height, self.robotData.fr_height,
                                                            self.robotData.bl_height, self.robotData.br_height])
        wheel_speeds = self.kinematicModel.getWheelVelocities()
        swerve_wheel_angles = self.kinematicModel.getSwerveWheelAngles()
        suspension_angles = self.kinematicModel.getSuspensionAngles()

        total_length = len(wheel_speeds) + len(swerve_wheel_angles) + len(suspension_angles)
        output_data = [0.0 for i in range(total_length)]
        for idx, ws, wa, sa in enumerate(zip(wheel_speeds, swerve_wheel_angles, suspension_angles)):
            output_data[idx] = ws
            output_data[idx + 4] = wa
            output_data[idx + 5] = sa
        return output_data

# pwms
#   'linear': pwm to assign all motors
#   'angular': pwm to assign all wheel servos
#   'height': pwm to assign all suspension servos
def PWM_to_RobotData(robot: RobotData, pwms: Dict, pwm_to_vel: Callable, pwm_to_theta: Callable):
    robot.linear = pwm_to_vel(pwms['linear'])
    robot.angular = pwm_to_theta(pwms['angular'])
