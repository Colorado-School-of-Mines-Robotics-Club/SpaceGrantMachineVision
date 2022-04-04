import threading
from typing import List, Tuple

try:
    import RPi.GPIO as GPIO
    from smbus2 import *
except ModuleNotFoundError:
    pass
except ImportError:
    pass


class HardwareManager:
    def __init__(self):
        # declare the bus
        self.bus = SMBus(1)

        # the values used by the example
        POWER_CTL = 0x2D
        MEASURE = 0x08
        BW_RATE = 0x2C
        DATA_FORMAT = 0x31

        self.address = 0x00  # made to make following lines work

        # enable measurement
        self.bus.write_byte_data(self.address, POWER_CTL, MEASURE)
        # set bandwith rate
        rate_flag = 0x0B
        self.bus.write_byte_data(self.address, BW_RATE, rate_flag)
        # set the measurement range for 10-bit readings
        range_flag = 0x00
        value = self.bus.read_byte_data(self.address, DATA_FORMAT)
        value &= ~0x0F
        value |= range_flag
        value |= 0x08
        self.bus.write_byte_data(self.address, DATA_FORMAT, value)

        self.pwm_address = 0x00
        self.accelerometer_address = 0x68

        # TODO
        # save time slice info to be returned with this????
        self.curr_motors = [0, 0, 0, 0]

        self.curr_encoders = [[0,0], [0,0], [0,0], [0,0]]
        self.past_encoders = [0,0,0,0]

        self.curr_servos = [0, 0, 0, 0, 0, 0, 0, 0]
        self.past_servos = [0, 0, 0, 0, 0, 0, 0, 0]

        self.curr_accel = [0, 0, 0]
        self.past_accel = [0, 0, 0]

        self.curr_gyro = [0, 0, 0]
        self.past_gyro = [0, 0, 0]

        self.motor_pins = [[11, 13], [31, 33], [36, 38], [16, 18]]
        self.servo_pins = [19, 21, 23, 37, 22, 24, 26, 32]
        self.dir_pins = [[8, 10], [19, 21], [22, 24], [27, 29]]

        self.motor_reg = [[7,6,9,8],[11,10,13,12],[15,14,17,16],[19,18,21,20]]
        self.servo_reg = [[23,22,25,24],[27,26,29,28],[31,30,33,32],[35,34,37,36],[39,38,41,40],[43,42,45,44],[47,46,49,48],[51,50,53,52]]
        self.accel_reg = 0x3B
        self.led_reg = [[55,54,57,56],[59,58,61,60],[63,62,65,64],[67,66,69,68]]

        # Split encoder reading into 4 threads for speed and accuracy
        self.motor_threads = [threading.Thread(target=self.read_motor, args=(i,)) for i in range(4)]

        # one thread for reading servo return data
        self.servos = threading.Thread(target=self.read_servo)

        # one thread for reading accelerometer data
        self.accel = threading.Thread(target=self.read_accelerometer)

    def start_threads(self) -> 'HardwareManager':
        # start all data collection threads
        for thread in self.motor_threads:
            thread.start()
        self.servos.start()
        self.accel.start()

        return self

    def join_threads(self) -> 'HardwareManager':
        # stops all data collection threads
        for thread in self.motor_threads:
            thread.join()
        self.servos.join()
        self.accel.join()

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
        directions, writes = HardwareManager.writes_convert(writes)
        self.write_pwm(directions, writes)

    # writes is a List[m1, m2, m3, m4, s1, s2, s3, s4, s5, s6, s7, s8, l1, l2, l3, l4]
    # motors are constrained to: [0, 4095]
    # servos are constrained to: [0, 4095]
    # leds are constrained to: [0, 4095]
    def write_pwm(self, directions: List[int], writes: List[int]):
        writes_counter = 0

        # Write PWM for each motor
        writes_counter = self.write_pwm_helper(self.motor_reg, writes_counter, writes)
        # Write PWM for each servo
        writes_counter = self.write_pwm_helper(self.servo_reg, writes_counter, writes)
        # Write PWM for each LED
        writes_counter = self.write_pwm_helper(self.led_reg, writes_counter, writes)

        self.write_gpio(directions)

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
            self.bus.write_i2c_block_data(self.pwm_address, reg[0], high1)
            # second register is the second byte of high
            self.bus.write_i2c_block_data(self.pwm_address, reg[1], high2)
            # 3rd register is the first byte of low
            self.bus.write_i2c_block_data(self.pwm_address, reg[2], low1)
            # 4th register is the second byte of low
            self.bus.write_i2c_block_data(self.pwm_address, reg[3], low2)

            # increment the writes_counter by one; only one value in the writes array was used
            writes_counter += 1
            # finished loop, continue to next register

        # return the updated writes_counter for use in the other calls
        return writes_counter

    def write_gpio(self, directions: List[int]):
        # iterate over the directions
        for i, direction in enumerate(directions):
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
                    # TODO: find the angle per click and add this instead of keeping track of number of "clicks"
                    self.curr_motors[thread] += 1
                else:
                    self.curr_motors[thread] -= 1

            self.past_encoders[thread] = self.curr_encoders[thread][0]

    def read_servo(self):
        # Read the new servo values, and store in curr_servo array. Save the original value to the past_servo
        while True:
            self.past_servos = self.curr_servos
            self.curr_servos = [GPIO.input(i) for i in range(8)]

    def read_accelerometer(self):
        # setup the accelerometer
        self.bus.write_byte_data(self.accelerometer_address, 0x19, 7)  # set sample rate
        self.bus.write_byte_data(self.accelerometer_address, 0x6B, 1)  # set power management
        self.bus.write_byte_data(self.accelerometer_address, 0x1A, 0)  # Set config
        self.bus.write_byte_data(self.accelerometer_address, 0x1B, 24)  # set gyro config
        self.bus.write_byte_data(self.accelerometer_address, 0x38, 1)  # interrupt enable register
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
