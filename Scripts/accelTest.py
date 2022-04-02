from smbus2 import *

sized = [0,0,0,0,0,0]
bus = SMBus(1)
accelerometer_address = 0x68
accel_reg = 0x3B

bus.write_byte_data(accelerometer_address, 0x19, 7) #set sample rate
bus.write_byte_data(accelerometer_address, 0x6B, 1) #set power management
bus.write_byte_data(accelerometer_address, 0x1A, 0) #Set config
bus.write_byte_data(accelerometer_address, 0x1B, 24) #set gyro config
bus.write_byte_data(accelerometer_address, 0x38, 1) #interrupt enable register


while True:
    data = bus.read_i2c_block_data(accelerometer_address, accel_reg, 12)
    for i in range(6):
        sized[i] =  (data[(2*i)]<<8) | (data[(2*i)+1])
    print(sized)