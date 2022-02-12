class HardwareManager:


    def __init__(self,address):
        try:
            import RPi.GPIO as GPIO
            self.GPIO_fail = False
        except ImportError:
            self.GPIO_fail = True
            pass

        self.address = address
        self.curr_motors = [0, 0, 0, 0]
        self.past_motors = [0, 0, 0, 0]

        self.curr_encoders = [[0,0], [0,0], [0,0], [0,0]]
        self.past_encoders = [[0,0], [0,0], [0,0], [0,0]]

        self.curr_servos = [0, 0, 0, 0, 0, 0, 0, 0]
        self.past_servos = [0, 0, 0, 0, 0, 0, 0, 0]

        self.curr_accel = [0, 0, 0]
        self.past_accel = [0, 0, 0]

        self.curr_gyro = [0, 0, 0]
        self.past_gyro = [0, 0, 0]

        self.motor_pins = [[11, 13], [15, 36], [38, 12], [16, 18]]
        self.servo_pins = [19, 21, 23, 37, 22, 24, 26, 32]
        self.dir_pins = [29, 31, 33, 35]

    def process_encoders(self, curr):

         #process encoder data on rising/falling edge

    def write_pwm(self, motor1, motor2, motor3, motor4, sus1, sus2, sus3, sus4, wheel1, wheel2, wheel3, wheel4, dir1,
                  dir2, dir3, dir4, rgb):
        #TODO: Write PWM for each motor

        #TODO: Write PWM for each servo
        #TODO: write PWM for each LED
        self.write_GPIO(self, dir1, dir2,dir3,dir4)

    def write_GPIO(self, dir1, dir2, dir3, dir4):
        dir1 = 1
        #TODO: write the output pins for the directions

    def read_motor(self):
        while True:
            self.curr_encoders[0][0] = GPIO.input(self.motor_pins[0][0])
            self.curr_encoders[0][1] = GPIO.input(self.motor_pins[0][1])
            self.curr_encoders[1][0] = GPIO.input(self.motor_pins[1][0])
            self.curr_encoders[1][1] = GPIO.input(self.motor_pins[1][1])
            self.curr_encoders[2][0] = GPIO.input(self.motor_pins[2][0])
            self.curr_encoders[2][1] = GPIO.input(self.motor_pins[2][1])
            self.curr_encoders[3][0] = GPIO.input(self.motor_pins[3][0])
            self.curr_encoders[3][1] = GPIO.input(self.motor_pins[3][1])
            self.process_encoders(1)


    def read_servo(self):
        self.curr_servo[0] = 1

    def read_accelerometer(self):
        self.curr_accel[0] = 1

    def get_data(self):
        return self.motors
        #TODO: return all data in a reasonable format (Maybe convert accelerometer data to velocity ect)

