from smbus2 import *

bus = SMBus(1)
accelerometer_address = 0x68
accel_reg = 0x32

while True:
    data = bus.read_i2c_block_data(accelerometer_address, accel_reg, 6)
    print(data)
