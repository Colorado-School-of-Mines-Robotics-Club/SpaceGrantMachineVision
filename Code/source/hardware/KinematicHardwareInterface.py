from typing import Dict, Callable, List, Union
import math

try:
    from .RobotData import RobotData
    from source.utilities.Config import Config
    from source.kinematics.KinematicModel import KinematicModel
    from source.kinematics.SimpleKinematicModel import SimpleKinematicModel
except ModuleNotFoundError as e:
    try:
        from .RobotData import RobotData
        from Code.source.utilities.Config import Config
        from Code.source.kinematics.KinematicModel import KinematicModel
        from Code.source.kinematics.SimpleKinematicModel import SimpleKinematicModel
    except ModuleNotFoundError:
        raise e


class KinematicHardwareInterface:
    def __init__(self, robotData: Union[RobotData, None] = None, ledStates: Union[List[bool], None] = None) -> None:
        self.kinematicModel = SimpleKinematicModel()# KinematicModel()
        self.robotData = RobotData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if robotData is not None:
            self.robotData = robotData
        self.ledStates = [False, False, False, False]
        if ledStates is not None:
            assert len(ledStates) == 4
            self.ledStates = ledStates
        self.motorServo = None
        self.command = None

        Config.init()
        self.max_pwm = Config.getElectronicPortsDict()['utility']['max_pwm']
        self.max_vel = Config.getElectronicPortsDict()['utility']['max_vel']
        self.max_rad = Config.getElectronicPortsDict()['utility']['max_rad']
        trim_dict = Config.getServoTrimDict()
        self.trim_values = []
        for key in trim_dict:
            self.trim_values.append(trim_dict[key])
        self.max_servo_pwm = Config.getElectronicPortsDict()['utility']['max_servo_pwm']
        self.min_servo_pwm = Config.getElectronicPortsDict()['utility']['min_servo_pwm']
        self.servo_center = int(((self.max_servo_pwm - self.min_servo_pwm) / 2) + self.min_servo_pwm)
        self.max_servo_dev = self.max_servo_pwm - self.servo_center

        self.updateFromRobotData()  # sets up the initial command

    def ms_to_pwm(self, ms: float) -> int:
        sign = -1 if ms < 0.0 else 1
        if abs(ms) >= self.max_vel:
            return sign * self.max_pwm
        return sign * int((abs(ms) / self.max_vel) * self.max_pwm)

    def radians_to_pwm(self, rad: float) -> int:
        sign = -1 if rad < 0.0 else 1
        rad = sign * self.max_rad if abs(rad) > self.max_servo_pwm else rad
        servo_dev = (rad / self.max_rad) * self.max_servo_dev
        servo_dev = servo_dev if abs(servo_dev) <= self.max_servo_dev else sign * self.max_servo_dev
        return self.servo_center + servo_dev

    def radians_to_deg(self, rad: float) -> int:
        return int(rad * 180.0 / math.pi)

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
                                   swerveAngles=[(self.robotData.angular + self.trim_values[i * 2] * math.pi / 180.0) for i in range(4)],
                                   suspensionHeightTargets=[self.robotData.fl_height + self.trim_values[1], self.robotData.fr_height + self.trim_values[3],
                                                            self.robotData.bl_height + self.trim_values[5], self.robotData.br_height + self.trim_values[7]])
        wheel_speeds = self.kinematicModel.getWheelVelocities()
        
        swerve_wheel_angles = self.kinematicModel.getSwerveWheelAngles()
        # print(f"Angles: {swerve_wheel_angles}")
        suspension_angles = self.kinematicModel.getSuspensionAngles()

        total_length = len(wheel_speeds) + len(swerve_wheel_angles) + len(suspension_angles)
        output_data = [0.0 for i in range(total_length)]
        for idx, data in enumerate(zip(wheel_speeds, swerve_wheel_angles, suspension_angles)):
            ws, wa, sa = data
            output_data[idx] = ws
            output_data[2 * idx + 4] = wa
            output_data[2 * idx + 5] = sa

        final_data = [0.0 for i in range(total_length)]
        final_data[0] = output_data[0] * -1 # front
        final_data[1] = output_data[1] * -1 # front
        final_data[2] = output_data[2] # back
        final_data[3] = output_data[3] # back
        final_data[4] = 90
        final_data[5] = 90
        final_data[6] = self.reverse_offset(output_data[4]) # front right
        final_data[7] = output_data[6] - 15 # back right
        final_data[8] = output_data[8] # back left
        final_data[9] = self.reverse_offset(output_data[10]) # front left
        final_data[10] = 90
        final_data[11] = 90

        con = 16

        final_data[6] = self.constrain(final_data[6], 90 - con, 90 + con)
        final_data[7] = self.constrain(final_data[7], 90 - 15 - con, 90 - 15 + con)
        final_data[8] = self.constrain(final_data[8], 90 - con, 90 + con)
        final_data[9] = self.constrain(final_data[9], 90 - con, 90 + con)

        self.motorServo = final_data
        self.command = self.motorServo + self.ledStates
    
    @staticmethod
    def reverse_offset(val):
        return 90 - (val - 90)

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
    
    def getCommandTargets(self) -> List[int]:
        motor_pwms = [self.ms_to_pwm(com) for com in self.command[0:4]]
        servo_angles = self.command[4:12]
        led_pwms = [self.bool_to_pwm(com) for com in self.ledStates]
        command = motor_pwms + servo_angles + led_pwms
        assert len(command) == 16
        return command

    @staticmethod
    def constrain(N: float, minN: Union[float, None] = None, maxN: Union[float, None] = None) -> float:
        if minN is not None and maxN is not None:
            return minN if N < minN else maxN if N > maxN else N
        return N
