# Built in python libs
import threading
from typing import List, Tuple, Union
import time

# Additional libs
import cv2
import numpy as np

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    from smbus2 import *
    from digi.xbee.devices import XBeeDevice
except ModuleNotFoundError:
    pass
except ImportError:
    pass

# Custom imports
try:
    from source.logger.Logger import Logger
    from .PIDController import PIDController
    from source.utilities.Config import Config
except ModuleNotFoundError as e:
    try:
        from Code.source.logger.Logger import Logger
        from .PIDController import PIDController
        from Code.source.utilities.Config import Config
    except ModuleNotFoundError:
        raise e


class HardwareManager:
    def __init__(self, feedback=True):
        self.feedback = feedback
        Config.init()
        electronics = Config.getElectronicPortsDict()
        motors = electronics['motors']
        servos = electronics['servos']
        leds = electronics['leds']
        sensors = electronics['sensors']
        accelerometer = sensors['accelerometer']
        xbee = sensors['xbee']
        poll_rates = electronics['poll_rates']
        utility = electronics['utility']
        pwm_board = electronics['pwm_board']

        # clicks per rev for motor
        self.clicks_per_rev = utility['clicks_per_rev']
        self.deg_per_click = 360.0 / self.clicks_per_rev

        # declare the bus
        self.bus = SMBus(1)

        # declare the PWM Controller and last sent PWM values
        # TODO
        # load constraint and setpoint data from config file for the PID controller
        self.pid: PIDController = PIDController(passthrough=True)
        self.targets = self.pid.get_targets()
        self.directions = [0, 0, 0, 0]

        # TODO
        # these need to be the actual PWMs that the servos and motors have last been read from their read methods
        # Since those read methods do not nessecarily read PWMs the conversions can occur in time with the PWM
        # controller thread. This alleviates the computational cost from the strictly reading threads.
        # This allows more time waiting on the bus, for more accurate values.
        # This ultimtely means a conversion function will need to be used inside of write_pwm_targets on the arguments
        # to the self.pid.get_pwm method
        self.curr_motor_pwm = [0, 0, 0, 0]
        self.curr_servo_pwm = [0, 0, 0, 0, 0, 0, 0, 0]

        # this can be assumed to always be the led_target
        self.curr_led_pwm = [0, 0, 0, 0]
        self.dt = 0.0

        self.pwm_address = pwm_board['address']

        self.accelerometer_address = accelerometer['address']
        self.accel_reg = accelerometer['register']
        self.accel_poll_rate = poll_rates['accelerometer']

        self.xbee_com = xbee['com_port']
        self.xbee_baudrate = xbee['baudrate']
        self.xbee_poll_rate = poll_rates['xbee']

        self.motor_pins = [motors['front_left']['enc_pins'], motors['front_right']['enc_pins'],
                           motors['back_left']['enc_pins'], motors['back_right']['enc_pins']]

        self.servo_pins = [servos['front_left_wheel']['pin'], servos['front_left_suspension']['pin'],
                           servos['front_right_wheel']['pin'], servos['front_right_suspension']['pin'],
                           servos['back_left_wheel']['pin'], servos['back_left_suspension']['pin'],
                           servos['back_right_wheel']['pin'], servos['back_right_suspension']['pin']]

        self.dir_pins = [motors['front_left']['dir_pins'], motors['front_right']['dir_pins'],
                         motors['back_left']['dir_pins'], motors['back_right']['dir_pins']]

        self.motor_reg = [motors['front_left']['registers'], motors['front_right']['registers'],
                          motors['back_left']['registers'], motors['back_right']['registers']]

        self.servo_reg = [servos['front_left_wheel']['registers'], servos['front_left_suspension']['registers'],
                          servos['front_right_wheel']['registers'], servos['front_right_suspension']['registers'],
                          servos['back_left_wheel']['registers'], servos['back_left_suspension']['registers'],
                          servos['back_right_wheel']['registers'], servos['back_right_suspension']['registers']]

        self.led_reg = [leds['one'], leds['two'], leds['three'], leds['four']]

        self.curr_motors = [0, 0, 0, 0]
        self.init_time = time.perf_counter()
        self.motor_time_slices: List[float] = [self.init_time, self.init_time, self.init_time, self.init_time]

        self.curr_encoders = [[0, 0], [0, 0], [0, 0], [0, 0]]
        self.past_encoders = [0, 0, 0, 0]

        self.curr_servos = [0, 0, 0, 0, 0, 0, 0, 0]
        self.past_servos = [0, 0, 0, 0, 0, 0, 0, 0]

        self.curr_accel = [0, 0, 0]
        self.past_accel = [0, 0, 0]

        self.curr_gyro = [0, 0, 0]
        self.past_gyro = [0, 0, 0]

        self.past_xbee = None
        self.curr_xbee = None

        # Split encoder reading into 4 threads for speed and accuracy
        self.motor_threads = [threading.Thread(target=self.read_motor, args=(i,), daemon=True) for i in range(4)]
        # one thread for reading servo return data
        self.servos = threading.Thread(target=self.read_servo, args=(), daemon=True)
        # one thread for reading accelerometer data
        self.accel = threading.Thread(target=self.read_accelerometer, args=(self.accel_poll_rate,), daemon=True)
        # thread for writing to the PWM breakout board at a certain HZ
        self.pwm_thread = threading.Thread(target=self.write_pwm_targets, args=(None,), daemon=True)
        # thread for reading the xbee
        self.xbee_thread = threading.Thread(target=self.read_xbee, args=(self.xbee_poll_rate,), daemon=True)

        # set the PWM board to 50hz
        self.bus.write_byte_data(self.pwm_address, 0x00, 0x00)
        self.bus.write_byte_data(self.pwm_address, 0x01, 0x18)
        self.bus.write_byte_data(self.pwm_address, 0xFE, 0x7A)

    def start_threads(self) -> 'HardwareManager':
        # start all data collection threads
        if self.feedback:
            for thread in self.motor_threads:
                thread.start()
            self.servos.start()
            self.accel.start()
            self.xbee_thread.start()
        self.pwm_thread.start()
        return self

    def join_threads(self) -> 'HardwareManager':
        # stops all data collection threads
        if self.feedback:
            for thread in self.motor_threads:
                thread.join()
            self.servos.join()
            self.accel.join()
            self.xbee_thread.join()
        self.pwm_thread.join()
        return self

    @staticmethod
    def writes_convert(writes: List[int], dirs=None) -> Tuple[List[int], List[int]]:
        if dirs is None:
            dirs = [0, 1, 0, 1]
        for i in range(4):  # only 4 motors and they appear at the front of writes
            if writes[i] < 0:
                dirs[i] = 1 if dirs[i] == 0 else 0
            elif writes[i] == 0:
                dirs[i] = 2
        writes = [abs(x) for x in writes]
        return dirs, writes

    def write_pwm_autodir(self, writes: List[int]):
        self.directions, writes = HardwareManager.writes_convert(writes)
        self.write_pwm(writes)

    def write_pwm_targets(self, hz: Union[float, None] = None):
        while True:
            _, _, self.curr_led_pwm = self.pid.get_targets_split()  # assume leds are at target always
            self.write_pwm(self.pid.get_pwm(self.curr_motor_pwm + self.curr_servo_pwm + self.curr_led_pwm))
            if hz is not None:
                time.sleep(1.0 / hz)

    # writes is a List[m1, m2, m3, m4, s1, s2, s3, s4, s5, s6, s7, s8, l1, l2, l3, l4]
    # motors are constrained to: [0, 4095]
    # servos are constrained to: [0, 4095]
    # leds are constrained to: [0, 4095]
    def write_pwm(self, writes: List[int]):
        writes_counter = 0

        # Write PWM for each motor
        writes_counter = self.write_pwm_helper(self.motor_reg, writes_counter, writes)
        # Write PWM for each servo
        writes_counter = self.write_pwm_helper(self.servo_reg, writes_counter, writes)
        # Write PWM for each LED
        writes_counter = self.write_pwm_helper(self.led_reg, writes_counter, writes)

        self.write_gpio()

    # writes the registers, then returns the current value of the writes_counter for later use
    def write_pwm_helper(self, reg_list, writes_counter, writes):
        for reg in reg_list:
            # four writes for each register
            # writes holds time high, use it to calculate time low
            high = writes[writes_counter]
            low = 4095 - high

            # calculate 1st byte for high, high is a number between 0-4095 (which takes up 12 bits max)
            # 1st bit holds biggest 4 bits of high, 2nd bit holds last 8 (4 + 8 = 12 bits stored)
            # an int is stored using 4 bits, so to isolate those 4, shift 20 left, then 28 right
            high1 = high
            high1 = (high1 << 20) >> 28
            # high1 is now the first 4 isolated bits
            # high2 needs to be the last 8 bits isolated, so shift left 24, right 24 to get rid of all leading bits
            high2 = high
            high2 = (high2 << 24) >> 24

            # same process for calculating the low data
            low1 = low
            low1 = (low1 << 20) >> 28

            low2 = low
            low2 = (low2 << 24) >> 24

            # write the 4 registers, data = [reg, byte_to_write]
            # first register is the first byte of high
            self.bus.write_i2c_block_data(self.pwm_address, reg[0], [high1])
            # second register is the second byte of high
            self.bus.write_i2c_block_data(self.pwm_address, reg[1], [high2])
            # 3rd register is the first byte of low
            self.bus.write_i2c_block_data(self.pwm_address, reg[2], [low1])
            # 4th register is the second byte of low
            self.bus.write_i2c_block_data(self.pwm_address, reg[3], [low2])

            # increment the writes_counter by one; only one value in the writes array was used
            writes_counter += 1
            # finished loop, continue to next register

        # return the updated writes_counter for use in the other calls
        return writes_counter

    def write_gpio(self):
        # iterate over the directions
        for i, direction in enumerate(self.directions):
            pin1, pin2 = self.dir_pins[i]
            dir1, dir2 = 0, 0
            if direction == 0:
                dir1, dir2 = 1, 0
            elif direction == 1:
                dir1, dir2 = 0, 1
            GPIO.output(pin1, dir1)
            GPIO.output(pin2, dir2)

    def read_motor(self, thread):
        # read current edge value before starting the loop
        self.past_encoders[thread] = GPIO.input(self.motor_pins[thread][0])

        while True:
            # read current encoder values
            self.curr_encoders[thread][0] = GPIO.input(self.motor_pins[thread][0])
            self.curr_encoders[thread][1] = GPIO.input(self.motor_pins[thread][1])

            # if edge detected, check B line for rotation direction
            if self.curr_encoders[thread][0] == self.past_encoders[thread]:
                if self.curr_encoders[thread][1] == self.curr_encoders[thread][0]:
                    # increment motor angle by angular distance of one click
                    self.curr_motors[thread] += self.deg_per_click
                else:
                    self.curr_motors[thread] -= self.deg_per_click

            self.past_encoders[thread] = self.curr_encoders[thread][0]
            self.motor_time_slices[thread] = time.perf_counter() - self.motor_time_slices[thread]

    def read_servo(self):
        # Read the new servo values, and store in curr_servo array. Save the original value to the past_servo
        while True:
            self.past_servos = self.curr_servos
            self.curr_servos = [GPIO.input(i) for i in range(8)]

    def init_accelerometer(self):
        self.bus.write_byte_data(self.accelerometer_address, 0x19, 7)  # set sample rate
        self.bus.write_byte_data(self.accelerometer_address, 0x6B, 1)  # set power management
        self.bus.write_byte_data(self.accelerometer_address, 0x1A, 0)  # Set config
        self.bus.write_byte_data(self.accelerometer_address, 0x1B, 24)  # set gyro config
        self.bus.write_byte_data(self.accelerometer_address, 0x38, 1)  # interrupt enable register

    def read_accelerometer(self, hz=240.0):
        # setup the accelerometer
        self.init_accelerometer()
        # temp area to store data
        sized = [0, 0, 0, 0, 0, 0]
        # Read the new accelerometer values, and store in curr_accel array. Save the original value to the past_accel
        while True:
            data = self.bus.read_i2c_block_data(self.accelerometer_address, self.accel_reg, 12)
            for i in range(6):
                sized[i] = (data[(2 * i)] << 8) | (data[(2 * i) + 1])

            # TODO
            # change from goofy analog numbers to real units (m/s^2 and degress/radians)

            # assign store data to class
            self.past_gyro = self.curr_gyro
            self.curr_gyro = sized[0:3]
            self.past_accel = self.curr_accel
            self.curr_accel = sized[3:len(sized)]

            time.sleep(1.0 / hz)

    def read_xbee(self, hz=240.0):
        try:
            xbee = XBeeDevice(self.xbee_com, self.xbee_baudrate)

            while True:
                self.past_xbee.append(self.curr_xbee)

                xbee_message = xbee.read_data()

                remote = xbee_message.remote_device
                data = xbee_message.data
                is_broadcast = xbee_message.is_broadcast
                timestamp = xbee_message.timestamp
                self.curr_xbee = (remote, data, is_broadcast, timestamp)

                time.sleep(1.0 / hz)

        except Exception:  # TODO well define the exception and test above code
            pass

    ''' 
    Decided to break this method into several different methods
    def get_data(self):
        return self.motors
        #TODO: return all data in a reasonable format
    '''

    # Get the information on the motors
    def get_curr_motors(self):
        return self.curr_motors

    # Get the information on the servos
    def get_past_servos(self):
        return self.past_servos

    def get_curr_servos(self):
        return self.curr_servos

    # Get the information on the accelerometer
    def get_past_accel(self):
        return self.past_accel

    def get_curr_accel(self):
        return self.curr_accel

    # Get the information on the gyro
    def get_past_gyro(self):
        return self.past_gyro

    def get_curr_gyro(self):
        return self.curr_gyro

    # PID methods
    # method to expose PIDController function update_targets
    def update_pwm_targets(self, targets: List[int]):
        self.directions, self.targets = HardwareManager.writes_convert(targets)
        self.pid.update_targets(self.targets)
