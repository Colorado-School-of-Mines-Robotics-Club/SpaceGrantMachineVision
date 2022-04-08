# Built in python libs
from typing import List, Tuple, Any, Union

# Additional libs
from simple_pid import PID

# Custom  imports


# in HardwareManager will need to keep track of a dt time to targets.
# values returned by the PID update functions will be the new PWM values
class PIDController:
    def __init__(self, mkp=5.0, mki=1.0, mkd=1.0, motor_set=0.0, motor_st=0.05, motor_limits=(0, 4095),
                 skp=5.0, ski=5.0, skd=1.0, servo_set=0.0, servo_st=0.05, servo_limits=(0, 4095),
                 num_motors=4, num_servos=8, num_leds=4):
        self.num_motors = num_motors
        self.num_servos = num_servos
        self.num_leds = num_leds
        self.total_size = self.num_motors + self.num_servos + self.num_leds
        self._s1, self._s2 = self.num_motors, self.num_motors + self.num_servos
        self.motor_controllers = [PID(mkp, mki, mkd, motor_set, motor_st, motor_limits) for i in range(num_motors)]
        self.servo_controllers = [PID(skp, ski, skd, servo_set, servo_st, servo_limits) for i in range(num_servos)]
        self.motor_targets = [0 for i in range(self.num_motors)]
        self.servo_targets = [0 for i in range(self.num_servos)]
        self.led_targets = [0 for i in range(self.num_leds)]

    def get_targets(self) -> List[int]:
        return self.motor_targets + self.servo_targets + self.led_targets

    def get_targets_split(self) -> Tuple[List[int], List[int], List[int]]:
        return self.motor_targets, self.servo_targets, self.led_targets

    def update_targets(self, targets=None) -> List[int]:
        if targets is not None:
            if len(targets) == self.total_size:
                motor_targets = targets[0:self._s1]
                servo_targets = targets[self._s1, self._s2]
                led_targets = targets[self._s2, self.total_size]
                self._update_targets(motor_targets, servo_targets, led_targets)
        return self.get_targets()

    def _update_targets(self, motor_targets=None, servo_targets=None, led_targets=None):
        if motor_targets is not None:
            if len(motor_targets) == self.num_motors:
                self.motor_targets = motor_targets
        if servo_targets is not None:
            if len(servo_targets) == self.num_servos:
                self.servo_targets = servo_targets
        if led_targets is not None:
            if len(led_targets) == self.num_leds:
                self.led_targets = led_targets

    def get_pwm(self, all_pwms=None) -> List[int]:
        if all_pwms is None:
            all_pwms = [0 for i in range(16)]
        else:
            assert len(all_pwms) == self.total_size
        motor_pwm, servo_pwm = self._get_motor_servo_pwm(all_pwms[0:self._s1], all_pwms[self._s1:self._s2])
        return motor_pwm + servo_pwm + self.led_targets

    def _get_motor_servo_pwm(self, current_motor_pwms=None, current_servo_pwms=None) -> Tuple[List[int], List[int]]:
        motor_pwms = self.motor_targets
        servo_pwms = self.servo_targets
        if current_motor_pwms is not None:
            motor_pwms = [int(pid(current_motor_pwms[i])) for i, pid in enumerate(self.motor_controllers)]
        if current_servo_pwms is not None:
            servo_pwms = [int(pid(current_servo_pwms[i])) for i, pid in enumerate(self.servo_controllers)]
        return motor_pwms, servo_pwms

    def update(self, current_pwms=None, targets=None) -> List[int]:
        self.update_targets(targets)
        return self.get_pwm(current_pwms)
