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
        #TODO: Write PWM for each motor

        #TODO: Write PWM for each servo
        #TODO: write PWM for each LED
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
        self.curr_servo[0] = 1

    def read_accelerometer(self):
        self.curr_accel[0] = 1

    def get_data(self):
        return self.motors
        #TODO: return all data in a reasonable format


