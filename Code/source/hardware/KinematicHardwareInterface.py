from typing import Dict, Callable, List, Union
import math

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
    def __init__(self, robotData: Union[RobotData, None] = None, ledStates: Union[List[bool], None] = None) -> None:
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

        Config.init()
        self.max_pwm = Config.getElectronicPortsDict()['utility']['max_pwm']
        self.max_vel = Config.getElectronicPortsDict()['utility']['max_vel']
        self.max_rad = Config.getElectronicPortsDict()['utility']['max_rad']

    def ms_to_pwm(self, ms: float) -> int:
        sign = -1 if ms < 0.0 else 1
        if abs(ms) >= self.max_vel:
            return sign * self.max_pwm
        return sign * int((abs(ms) / self.max_vel) * self.max_pwm)

    def radians_to_pwm(self, rad: float) -> int:
        sign = -1 if rad < 0.0 else 1
        if abs(rad) >= self.max_rad:
            return sign * self.max_pwm
        return sign * int((abs(rad) / self.max_rad) * self.max_pwm)

    @staticmethod
    def bool_to_pwm(boolean: bool) -> int:
        if boolean:
            return 4095
        return 0

    # Will take robotData parameters and run them through the kinematic model
    # will return m/s for motors and angles in radians for servos
    def updateFromRobotData(self, robotData: Union[RobotData, None] = None) -> None:
        if robotData is not None:
            self.robotData = robotData
        self.kinematicModel.update(wheelVelocities=[self.robotData.linear for i in range(4)],
                                   swerveAngles=[self.robotData.angular * math.pi / 180.0 for i in range(4)],
                                   suspensionHeightTargets=[self.robotData.fl_height, self.robotData.fr_height,
                                                            self.robotData.bl_height, self.robotData.br_height])
        wheel_speeds = self.kinematicModel.getWheelVelocities()
        swerve_wheel_angles = self.kinematicModel.getSwerveWheelAngles()
        suspension_angles = self.kinematicModel.getSuspensionAngles()

        total_length = len(wheel_speeds) + len(swerve_wheel_angles) + len(suspension_angles)
        output_data = [0.0 for i in range(total_length)]
        for idx, data in enumerate(zip(wheel_speeds, swerve_wheel_angles, suspension_angles)):
            ws, wa, sa = data
            output_data[idx] = ws
            output_data[2 * idx + 4] = wa
            output_data[2 * idx + 5] = sa
        self.motorServo = output_data
        self.command = self.motorServo + self.ledStates

    def updateFromState(self) -> None:
        # TODO
        pass

    # takes a List of length 4 with true false states
    def updateLEDs(self, ledStates: List[bool]):
        assert len(ledStates) == 4
        self.ledStates = ledStates
        self.command = self.motorServo + self.ledStates

    # gets the pwms for motors and servos from our interface, does not have leds
    def getCommandPWM(self) -> List[int]:
        motor_pwms = [self.ms_to_pwm(com) for com in self.command[0:4]]
        servo_pwms = [self.radians_to_pwm(com) for com in self.command[4:12]]
        led_pwms = [self.bool_to_pwm(com) for com in self.ledStates]
        command = motor_pwms + servo_pwms + led_pwms
        assert len(command) == 16
        return command