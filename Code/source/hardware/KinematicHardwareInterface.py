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
    def __init__(self, robotData: Union[RobotData, None] = None, ledStates: Union[List[bool], None] = None):
        self.kinematicModel = KinematicModel()
        self.robotData = RobotData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if robotData is not None:
            self.robotData = robotData
        self.ledStates = [False, False, False, False]
        if ledStates is not None:
            assert len(ledStates) == 4
            self.ledStates = ledStates
        self.motorServo = None
        self.command = None
        self.updateFromRobotData()  # sets up the initial command

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

    @staticmethod
    def bool_to_pwm(boolean: bool) -> int:
        if boolean:
            return 4095
        return 0

    @staticmethod
    def pwm_to_bool(pwm: int) -> bool:
        if pwm > 0:
            return True
        return False

    # Will take robotData parameters and run them through the kinematic model
    # will return m/s for motors and angles in radians for servos
    def updateFromRobotData(self, robotData: Union[RobotData, None] = None) -> None:
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
        self.motorServo = output_data
        self.command = self.motorServo + self.ledStates

    # takes a List of length 4 with true false states
    def updateLEDs(self, ledStates: List[bool]):
        assert len(ledStates) == 4
        self.ledStates = ledStates
        self.command = self.motorServo + self.ledStates

    # gets the pwms for motors and servos from our interface, does not have leds
    def getCommandPWM(self) -> List[int]:
        motor_pwms = [self.ms_to_pwm(com) for com in self.command[0:4]]
        servo_pwms = [self.radians_to_pwm(com) for com in self.command[4:-1]]
        led_pwms = [self.bool_to_pwm(com) for com in self.ledStates]
        return motor_pwms + servo_pwms + led_pwms

# pwms
#   'linear': pwm to assign all motors
#   'angular': pwm to assign all wheel servos
#   'height': pwm to assign all suspension servos
def PWM_to_RobotData(robot: RobotData, pwms: Dict, pwm_to_vel: Callable, pwm_to_theta: Callable):
    robot.linear = pwm_to_vel(pwms['linear'])
    robot.angular = pwm_to_theta(pwms['angular'])
