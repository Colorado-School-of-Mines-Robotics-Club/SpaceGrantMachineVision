# Built in python libs
import threading
from threading import Lock
from typing import List, Tuple, Union
import time
from collections import deque
import sys
import math

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
    def __init__(self, motor_feedback=True, servo_feedback=True, accel_feedback=False, xbee_feedback=False,
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

        self.commandQueue = deque()

        # define the PWM control board
        with self.i2c_lock:
            self.pca = PCA9685(busio.I2C(SCL, SDA))  # IDK if we should do any cleanup if this fails??
        self.pca.frequency = electronics['pwm_board']['hz']

        self.pid: PIDController = PIDController(passthrough=True)
        self.targets = self.pid.get_targets()
        
        # pins for motor encoders and direction setting
        self.motor_pins = [motors['front_left']['enc_pins'], motors['front_right']['enc_pins'],
                           motors['back_left']['enc_pins'], motors['back_right']['enc_pins']]
        self.dir_pins = [motors['front_left']['dir_pins'], motors['front_right']['dir_pins'],
                         motors['back_left']['dir_pins'], motors['back_right']['dir_pins']]
        self.num_motors = len(self.motor_pins)

        # servo feedback pins
        self.servo_pins = [servos['front_left_wheel']['pin'], servos['front_left_suspension']['pin'],
                           servos['front_right_wheel']['pin'], servos['front_right_suspension']['pin'],
                           servos['back_left_wheel']['pin'], servos['back_left_suspension']['pin'],
                           servos['back_right_wheel']['pin'], servos['back_right_suspension']['pin']]
        self.num_servos = len(self.servo_pins)

        with self.i2c_lock:
            self.servos = [servo.Servo(self.pca.channels[i + 4]) for i in range(self.num_servos)]

        # setup the registers for the leds
        self.led_reg = [leds['one'], leds['two'], leds['three'], leds['four']]

        for pin in self.dir_pins:
            GPIO.setup(pin, GPIO.OUT)
        for pin in self.servo_pins:
            GPIO.setup(pin, GPIO.IN)
        for pins in self.motor_pins:
            GPIO.setup(pins[0], GPIO.IN)
            GPIO.setup(pins[1], GPIO.IN)

        self.pwm_thread = threading.Thread(name=f"motor_controller_thread", target=self.motor_controller_loop, daemon=True)

    def start_threads(self):
        self.pwm_thread.start()
        return self
    
    def join_threads(self):
        self.pwm_thread.join()
    
    def shutdown(self):
        self.join_threads()
        time.sleep(0.1)
        self.pca.deinit()
        time.sleep(0.1)
        GPIO.cleanup()
        time.sleep(0.1)

    def motor_controller_loop(self):
        while not self.stopped:
            try:
                commandData = self.commandQueue.popleft()
            except IndexError:
                # commandData = ([0, 0, 0, 0, 90, 90, 90, 90, 90, 90, 90, 90, 0, 0, 0, 0], 0.1)
                continue
            command_targets, command_time = commandData
            commandStartTime = time.perf_counter()
            while time.perf_counter() - commandStartTime < command_time and not self.stopped:
                self.update_motors(command_targets)

    def update_motors(self, commandTargets: List):
        for i in range(4):
            pin1, pin2 = self.dir_pins[i]
            dir1, dir2 = 0, 0
            if commandTargets[i] > 0:
                dir1, dir2 = 1, 0
            elif commandTargets[i] < 0:
                dir1, dir2 = 0, 1
            else:
                dir1, dir2 = 0, 0
            GPIO.output(pin1, dir1)
            GPIO.output(pin2, dir2)

        for i in range(4):
            self.pca.channels[i].duty_cycle = abs(commandTargets[i])

        for i in range(0, 8):
            self.servos[i].angle = int(commandTargets[i + 4])
                
    # PID methods
    # method to expose PIDController function update_targets
    def update_targets(self, targets: List[Tuple[List[int], float]]):
        self.commandQueue.clear()
        self.commandList = targets
        for command in self.commandList:
            self.commandQueue.append(command)

    def stop(self):
        self.stopped = True
        self.join_threads()
        time.sleep(1)
        GPIO.cleanup()
