import threading


class HardwareManager:
    def __init__(self):
        try:
            import RPi.GPIO as GPIO
            self.GPIO_fail = False
        except ImportError:
            self.GPIO_fail = True
            pass

        self.pwm_address = 0x00
        self.accelerometer_address = 0x00

        self.curr_motors = [0, 0, 0, 0]
        self.past_motors = [0, 0, 0, 0]

        self.curr_encoders = [[0,0], [0,0], [0,0], [0,0]]
        self.past_encoders = [0,0,0,0]

        self.curr_servos = [0, 0, 0, 0, 0, 0, 0, 0]
        self.past_servos = [0, 0, 0, 0, 0, 0, 0, 0]

        self.curr_accel = [0, 0, 0]
        self.past_accel = [0, 0, 0]

        self.curr_gyro = [0, 0, 0]
        self.past_gyro = [0, 0, 0]

        self.motor_pins = [[11, 13], [15, 36], [38, 12], [16, 18]]
        self.servo_pins = [19, 21, 23, 37, 22, 24, 26, 32]
        self.dir_pins = [29, 31, 33, 35]

        #Split encoder reading into 4 threads for speed and accuracy
        motor1 = threading.Thread(target=self.read_motor, args=(0,))
        motor2 = threading.Thread(target=self.read_motor, args=(1,))
        motor3 = threading.Thread(target=self.read_motor, args=(2,))
        motor4 = threading.Thread(target=self.read_motor, args=(3,))

        #one thread for reading servo return data
        servos = threading.Thread(target=self.read_servo)

        #one thread for reading accelerometer data
        accel = threading.Thread(target=self.read_accelerometer)

        #start all data collection threads
        motor1.start()
        motor2.start()
        motor3.start()
        motor4.start()
        servos.start()

    def write_pwm(self, motor1, motor2, motor3, motor4, sus1, sus2, sus3, sus4, wheel1, wheel2, wheel3, wheel4, dir1,
                  dir2, dir3, dir4, rgb):
        writes = []

        writes_counter = 0
        #TODO: Write PWM for each motor
        for reg in motor_reg:
            #four writes for each register
            #writes holds time high, use it to calculate time low
            high = writes[writes_counter]
            low = 4095 - high

            #calculate 1st byte for high, high is a number between 0-4095 (which takes up 12 bits max)
            #1st bit holds biggest 4 bits of high, 2nd bit holds last 8 (4 + 8 = 12 bits stored)
            #an int is stored using 4 bits, so to isolate those 4, shift 20 left, then 28 right
            high1 = high
            high1 << 20
            high1 >> 28
            #high1 is now the first 4 isolated bits
            #high2 needs to be the last 8 bits isolated, so shift left 24, right 24 to get rid of all leading bits
            high2 = high
            high2 << 24
            high2 >> 24

            #same process for calculating the low data
            low1 = low
            low1 << 20
            low1 >> 28

            low2 = low
            low2 << 24
            low2 >> 24

            #write the 4 registers, data = [reg, byte_to_write]
            data = [reg[0][0], high1] #first register is the first byte of high
            bus.write_I2C_block_data(pwm_address, 0, data)
            data = [reg[0][1], high2]#second register is the second byte of high
            bus.write_IC2_block_data(pwm_address, 0, data)
            data = [reg[1][0], low1]#3rd register is the first byte of low
            bus.write_IC2_block_data(pwm_address, 0, data)
            data = [reg[1][1], low2]#4th register is the second byte of low
            bus.write_IC2_block_data(pwm_address, 0, data)

            #increment the writes_counter by one; only one value in the writes array was used
            writes_counter += 1

        #TODO: Write PWM for each servo
        for reg in servo_reg:
            # four writes for each register
            # writes holds time high, use it to calculate time low
            high = writes[writes_counter]
            low = 4095 - high

            # calculate 1st byte for high, high is a number between 0-4095 (which takes up 12 bits max)
            # 1st bit holds biggest 4 bits of high, 2nd bit holds last 8 (4 + 8 = 12 bits stored)
            # an int is stored using 4 bits, so to isolate those 4, shift 20 left, then 28 right
            high1 = high
            high1 << 20
            high1 >> 28
            # high1 is now the first 4 isolated bits
            # high2 needs to be the last 8 bits isolated, so shift left 24, right 24 to get rid of all leading bits
            high2 = high
            high2 << 24
            high2 >> 24

            # same process for calculating the low data
            low1 = low
            low1 << 20
            low1 >> 28

            low2 = low
            low2 << 24
            low2 >> 24

            # write the 4 registers, data = [reg, byte_to_write]
            data = [reg[0][0], high1]  # first register is the first byte of high
            bus.write_I2C_block_data(pwm_address, 0, data)
            data = [reg[0][1], high2]  # second register is the second byte of high
            bus.write_IC2_block_data(pwm_address, 0, data)
            data = [reg[1][0], low1]  # 3rd register is the first byte of low
            bus.write_IC2_block_data(pwm_address, 0, data)
            data = [reg[1][1], low2]  # 4th register is the second byte of low
            bus.write_IC2_block_data(pwm_address, 0, data)

            # increment the writes_counter by one; only one value in the writes array was used
            writes_counter += 1

        #TODO: write PWM for each LED
        for reg in LED_reg:
            # four writes for each register
            # writes holds time high, use it to calculate time low
            high = writes[writes_counter]
            low = 4095 - high

            # calculate 1st byte for high, high is a number between 0-4095 (which takes up 12 bits max)
            # 1st bit holds biggest 4 bits of high, 2nd bit holds last 8 (4 + 8 = 12 bits stored)
            # an int is stored using 4 bits, so to isolate those 4, shift 20 left, then 28 right
            high1 = high
            high1 << 20
            high1 >> 28
            # high1 is now the first 4 isolated bits
            # high2 needs to be the last 8 bits isolated, so shift left 24, right 24 to get rid of all leading bits
            high2 = high
            high2 << 24
            high2 >> 24

            # same process for calculating the low data
            low1 = low
            low1 << 20
            low1 >> 28

            low2 = low
            low2 << 24
            low2 >> 24

            # write the 4 registers, data = [reg, byte_to_write]
            data = [reg[0][0], high1]  # first register is the first byte of high
            bus.write_I2C_block_data(pwm_address, 0, data)
            data = [reg[0][1], high2]  # second register is the second byte of high
            bus.write_IC2_block_data(pwm_address, 0, data)
            data = [reg[1][0], low1]  # 3rd register is the first byte of low
            bus.write_IC2_block_data(pwm_address, 0, data)
            data = [reg[1][1], low2]  # 4th register is the second byte of low
            bus.write_IC2_block_data(pwm_address, 0, data)

            # increment the writes_counter by one; only one value in the writes array was used
            writes_counter += 1

        self.write_GPIO(self, dir1, dir2,dir3,dir4)

    def write_gpio(self, dir1, dir2, dir3, dir4):
        dir1 = 1
        #TODO: write the output pins for the directions


    def read_motor(self, thread):
        #read current edge value before starting the loop
        self.past_encoders[thread] = GPIO.input(self.motor_pins[thread][0])

        while True:
            #read current encoder values
            self.curr_encoders[thread][0] = GPIO.input(self.motor_pins[thread][0])
            self.curr_encoders[thread][1] = GPIO.input(self.motor_pins[thread][1])

            #if edge detected, check B line for rotation direction
            if self.curr_encoders[thread][0] == self.past_encoders[thread]:
                if self.curr_encoders[thread][1] == self.curr_encoders[thread][0] :
                    # increment motor angle by angular distance of one click
                    #TODO: find the angle per click and add this instead of keeping track of number of "clicks"
                    self.curr_motors[thread] += 1
                else:
                    self.curr_motors[thread] -= 1

            self.past_encoders[thread] = self.curr_encoders[thread][0]

    def read_servo(self):
        while True:
            self.curr_servo[0] = GPIO.input(self.servo_pins[0])
            self.curr_servo[1] = GPIO.input(self.servo_pins[1])
            self.curr_servo[2] = GPIO.input(self.servo_pins[2])
            self.curr_servo[3] = GPIO.input(self.servo_pins[3])
            self.curr_servo[4] = GPIO.input(self.servo_pins[4])
            self.curr_servo[5] = GPIO.input(self.servo_pins[5])
            self.curr_servo[6] = GPIO.input(self.servo_pins[6])
            self.curr_servo[7] = GPIO.input(self.servo_pins[7])
            

    def read_accelerometer(self):
        self.curr_accel[0] = 1

    def get_data(self):
        return self.motors
        #TODO: return all data in a reasonable format


