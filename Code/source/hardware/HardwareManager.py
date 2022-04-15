# Built in python libs
import threading
from threading import Lock
from typing import List, Tuple, Union
import time
from collections import deque
import sys

# Additional libs
import cv2
import numpy as np

try:
    import RPi.GPIO as GPIO
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    from smbus2 import *
    from digi.xbee.devices import XBeeDevice
    import board
    from board import SCL, SDA
    import busio
    from adafruit_motor import servo
    from adafruit_pca9685 import PCA9685
    from adafruit_mpu6050 import MPU6050
except ModuleNotFoundError:
    pass
except ImportError:
    pass
except NotImplementedError:
    pass

# Custom imports
try:
    from source.logger.Logger import Logger
    from .PIDController import PIDController
    from source.utilities.Config import Config
except ModuleNotFoundError as importException:
    try:
        from Code.source.logger.Logger import Logger
        from .PIDController import PIDController
        from Code.source.utilities.Config import Config
    except ModuleNotFoundError:
        raise importException


class HardwareManager:
    def __init__(self, motor_feedback=True, servo_feedback=True, accel_feedback=False, xbee_feedback=True,
                 instant_encoder_reads=True):
        # flag for stopping all threads gracefully
        self.stopped = False

        # lock for doing any operations on the I2C bus
        self.i2c_lock = Lock()

        Config.init()
        electronics = Config.getElectronicPortsDict()
        motors = electronics['motors']
        servos = electronics['servos']
        leds = electronics['leds']
        sensors = electronics['sensors']
        xbee = sensors['xbee']
        accel = sensors['accelerometer']
        poll_rates = electronics['poll_rates']
        utility = electronics['utility']

        # queue for commands to be in
        self.commandList = list()
        self.commandQueue = deque()

        # clicks per rev for motor

        # define the PWM control board
        with self.i2c_lock:
            self.pca = PCA9685(busio.I2C(SCL, SDA))  # IDK if we should do any cleanup if this fails??
        self.pca.frequency = electronics['pwm_board']['hz']

        # declare the PWM Controller and last sent PWM values
        # TODO
        # load constraint and setpoint data from config file for the PID controller
        self.pid: PIDController = PIDController(passthrough=True)
        self.targets = self.pid.get_targets()
        self.directions = motors['dir_setpoints']  # the default directions setup for forward
        self.pwm_write_poll_rate = poll_rates['writes']

        # setup the accel device
        try:
            if not accel_feedback:  # if we turn accel off we should just do nothing
                raise IOError
            with self.i2c_lock:
                self.accel = MPU6050(board.I2C())
        except IOError:
            self.accel = None
        finally:
            self.accel_poll_rate = poll_rates['accelerometer']
            self.accel_feedback = accel_feedback
            self.curr_accel = accel['accel_data']
            self.past_accel = accel['accel_data']
            self.curr_gyro = accel['gyro_data']
            self.past_gyro = accel['gyro_data']

        # setup the xbee device
        try:
            if not xbee_feedback:  # if we turn xbee off we should just do nothing
                raise IOError
            with self.i2c_lock:
                self.xbee = XBeeDevice(xbee['com_port'], xbee['baudrate'])
        except IOError:
            self.xbee = None
        finally:
            self.xbee_poll_rate = poll_rates['xbee']
            self.xbee_feedback = xbee_feedback
            self.past_xbee = xbee['data']
            self.curr_xbee = xbee['data']

        # pins for motor encoders and direction setting
        self.motor_pins = [motors['front_left']['enc_pins'], motors['front_right']['enc_pins'],
                           motors['back_left']['enc_pins'], motors['back_right']['enc_pins']]
        self.dir_pins = [motors['front_left']['dir_pins'], motors['front_right']['dir_pins'],
                         motors['back_left']['dir_pins'], motors['back_right']['dir_pins']]
        self.motor_reg = [motors['front_left']['registers'], motors['front_right']['registers'],
                          motors['back_left']['registers'], motors['back_right']['registers']]
        self.num_motors = len(self.motor_pins)
        self.motor_feedback = motor_feedback
        self.curr_encoders = [[0, 0] for i in range(self.num_motors)]
        self.past_encoders = [0 for i in range(self.num_motors)]
        self.clicks_per_rev = utility['clicks_per_rev']
        self.deg_per_click = 360.0 / self.clicks_per_rev
        self.init_time = time.perf_counter()
        self.motor_time_slices: List[float] = [self.init_time, self.init_time, self.init_time, self.init_time]
        self.curr_motor_pwm = [0 for i in range(self.num_motors)]  # TODO update from curr and past data
        self.curr_motor_enc = [0 for i in range(self.num_motors)]
        self.direction_setpoints = motors['dir_setpoints']
        self.motor_poll_rate = poll_rates['motors'] if not instant_encoder_reads else None

        # servo feedback pins
        self.servo_pins = [servos['front_left_wheel']['pin'], servos['front_left_suspension']['pin'],
                           servos['front_right_wheel']['pin'], servos['front_right_suspension']['pin'],
                           servos['back_left_wheel']['pin'], servos['back_left_suspension']['pin'],
                           servos['back_right_wheel']['pin'], servos['back_right_suspension']['pin']]
        self.servo_reg = [servos['front_left_wheel']['registers'], servos['front_left_suspension']['registers'],
                          servos['front_right_wheel']['registers'], servos['front_right_suspension']['registers'],
                          servos['back_left_wheel']['registers'], servos['back_left_suspension']['registers'],
                          servos['back_right_wheel']['registers'], servos['back_right_suspension']['registers']]
        self.num_servos = len(self.servo_pins)
        with self.i2c_lock:
            self.servos = [servo.Servo(self.pca.channels[i + 4]) for i in range(self.num_servos)]
        self.servo_feedback = servo_feedback
        self.curr_servo_pwm = [0 for i in range(self.num_servos)]  # TODO update from curr and past data
        self.curr_servos = [0 for i in range(self.num_servos)]
        self.past_servos = [0 for i in range(self.num_servos)]
        self.servo_poll_rate = poll_rates['servos']

        # setup the registers for the leds
        self.led_reg = [leds['one'], leds['two'], leds['three'], leds['four']]

        for pin in self.dir_pins:
            GPIO.setup(pin, GPIO.OUT)
        for pin in self.servo_pins:
            GPIO.setup(pin, GPIO.IN)
        for pins in self.motor_pins:
            GPIO.setup(pins[0], GPIO.IN)
            GPIO.setup(pins[1], GPIO.IN)

        # Split encoder reading into 4 threads for speed and accuracy at a certain HZ
        self.motor_threads = [threading.Thread(name=f"motor_{i}_thread", target=self.read_motor, args=(i, self.motor_poll_rate,), daemon=True) for i in range(self.num_motors)]
        # one thread for reading servo return data at a certain HZ
        self.servo_thread = threading.Thread(name=f"servo_thread", target=self.read_servo, args=(self.servo_poll_rate,), daemon=True)
        # one thread for reading accelerometer data at a certain HZ
        self.accel_thread = threading.Thread(name=f"accel_thread", target=self.read_accelerometer, args=(self.accel_poll_rate,), daemon=True)
        # thread for writing to the PWM breakout board at a certain HZ
        self.pwm_thread = threading.Thread(name=f"pwm_write_thread", target=self.write_pwm_targets, args=(self.pwm_write_poll_rate,), daemon=True)
        # thread for reading the xbee at a certain HZ
        self.xbee_thread = threading.Thread(name=f"xbee_thread", target=self.read_xbee, args=(self.xbee_poll_rate,), daemon=True)

    def start_threads(self) -> 'HardwareManager':
        # start all data collection threads
        if self.motor_feedback:
            for thread in self.motor_threads:
                thread.start()
        if self.servo_feedback:
            self.servo_thread.start()
        if self.accel_feedback:
            self.accel_thread.start()
        if self.xbee_feedback:
            self.xbee_thread.start()
        # starts the pwm writing thread
        self.pwm_thread.start()
        return self

    def join_threads(self) -> 'HardwareManager':
        # stops all data collection threads
        if self.motor_feedback:
            for thread in self.motor_threads:
                thread.join()
        if self.servo_feedback:
            self.servo_thread.join()
        if self.accel_feedback:
            self.accel_thread.join()
        if self.xbee_feedback:
            self.xbee_thread.join()
        # stops the pwm writing thread
        self.pwm_thread.join()
        self.pca.deinit()
        return self

    def writes_convert(self, writes: List[int], dirs=None) -> Tuple[List[int], List[int]]:
        if dirs is None:
            dirs = self.direction_setpoints
        for i in range(self.num_motors):  # only 4 motors and they appear at the front of writes
            if writes[i] < 0:
                dirs[i] = 1 if dirs[i] == 0 else 0
            elif writes[i] == 0:
                dirs[i] = 2
            writes[i] = abs(writes[i])  # only motors should be given a strictly positive value
        # writes = [abs(x) for x in writes]
        return dirs, writes

    def write_pwm_autodir(self, writes: List[int]):
        self.directions, writes = self.writes_convert(writes)
        self.write_pwm(writes)

    def write_pwm_targets(self, hz=240.0):
        while not self.stopped:
            try:
                commandData = self.commandQueue.popleft()
            except IndexError:
                continue
            command_pwm, command_time = commandData
            self.pid.update_targets(command_pwm)
            _, _, led_target = self.pid.get_targets_split()
            commandStartTime = time.perf_counter()
            while time.perf_counter() - commandStartTime < command_time and not self.stopped:
                self.directions, self.targets = self.writes_convert(command_pwm)
                self.pid.update_targets(self.targets)
                self.write_pwm(self.pid.get_pwm(self.curr_motor_pwm + self.curr_servo_pwm + led_target))  # writes gpio

                if hz is not None:
                    time.sleep(1.0 / hz)

    # writes is a List[m1, m2, m3, m4, s1, s2, s3, s4, s5, s6, s7, s8, l1, l2, l3, l4]
    # motors are constrained to: [0, 4095]
    # servos are constrained to: [0, 4095]
    # leds are constrained to: [0, 4095]
    def write_pwm(self, writes: List[int]):
        writes_counter = 0
        # Write PWM for each motor
        writes_counter = self.write_motors(writes_counter, writes)
        # Write angle for each servo
        _ = self.write_servos(writes_counter, writes)
        # write the GPIO pins for directions
        self.write_gpio()

    # writes the registers, then returns the current value of the writes_counter for later use
    def write_motors(self, writes_counter, writes):
        # iterate over the motors setting up the write
        for i in range(self.num_motors):
            with self.i2c_lock:
                self.pca.channels[i].duty_cycle = writes[writes_counter]
            writes_counter += 1

        # return the updated writes_counter for use in the other calls
        return writes_counter

    # writes the registers, then returns the current value of the writes_counter for later use
    def write_servos(self, writes_counter, writes):
        # iterate over the servos setting up the writes
        for i in range(self.num_servos):
            with self.i2c_lock:
                self.servos[i].angle = writes[writes_counter]
            writes_counter += 1

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

    def read_motor(self, thread, hz=60.0):
        # read current edge value before starting the loop
        self.past_encoders[thread] = GPIO.input(self.motor_pins[thread][0])

        while not self.stopped:
            # read current encoder values
            self.curr_encoders[thread][0] = GPIO.input(self.motor_pins[thread][0])
            self.curr_encoders[thread][1] = GPIO.input(self.motor_pins[thread][1])

            # if edge detected, check B line for rotation direction
            if self.curr_encoders[thread][0] == self.past_encoders[thread]:
                if self.curr_encoders[thread][1] == self.curr_encoders[thread][0]:
                    # increment motor angle by angular distance of one click
                    self.curr_motor_enc[thread] += self.deg_per_click
                else:
                    self.curr_motor_enc[thread] -= self.deg_per_click

            self.past_encoders[thread] = self.curr_encoders[thread][0]
            self.motor_time_slices[thread] = time.perf_counter() - self.motor_time_slices[thread]

            if hz is not None:
                time.sleep(1.0 / hz)

    def read_servo(self, hz=60.0):
        # Read the new servo values, and store in curr_servo array. Save the original value to the past_servo
        while not self.stopped:
            self.past_servos = self.curr_servos
            self.curr_servos = [GPIO.input(i) for i in range(self.num_servos)]

            if hz is not None:
                time.sleep(1.0 / hz)

    def read_accelerometer(self, hz=60.0):
        # Read the new accelerometer values, and store in curr_accel array. Save the original value to the past_accel
        while not self.stopped:
            self.past_gyro = self.curr_gyro
            self.past_accel = self.curr_accel
            with self.i2c_lock:
                self.curr_accel = self.accel.acceleration
                self.curr_gyro = self.accel.gyro

            if hz is not None:
                time.sleep(1.0 / hz)

    def read_xbee(self, hz=60.0):
        # read data from the xbee
        while True:
            self.past_xbee.append(self.curr_xbee)
            with self.i2c_lock:
                xbee_message = self.xbee.read_data()
            remote = xbee_message.remote_device
            data = xbee_message.data
            is_broadcast = xbee_message.is_broadcast
            timestamp = xbee_message.timestamp
            self.curr_xbee = (remote, data, is_broadcast, timestamp)

            if hz is not None:
                time.sleep(1.0 / hz)

    ''' 
    Decided to break this method into several different methods
    def get_data(self):
        return self.motors
        #TODO: return all data in a reasonable format
    '''

    # Get the information on the motors
    def get_curr_motors(self):
        return self.curr_motor_enc

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
    def update_pwm_targets(self, targets: List[Tuple[List[int], float]]):
        self.commandQueue.clear()
        self.commandList = targets
        for command in self.commandList:
            self.commandQueue.append(command)

    def stop(self):
        self.stopped = True
        self.join_threads()
