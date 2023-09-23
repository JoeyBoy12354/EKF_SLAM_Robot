import smbus
import time

# Define the I2C bus (1 for Odroid XU4)
bus = smbus.SMBus(1)

# LSM6DSOX I2C address
lsm6dsox_address = 0x6A  # Check the datasheet for the correct address

# Configure the sensor (you may need to adjust these settings)
bus.write_byte_data(lsm6dsox_address, 0x10, 0x40)  # CTRL1_XL register, set accelerometer to 2g
bus.write_byte_data(lsm6dsox_address, 0x11, 0x40)  # CTRL2_G register, set gyroscope to 2000 dps

# Read accelerometer and gyroscope data
while True:
    # Read accelerometer data (16-bit signed values)
    accel_x = bus.read_i2c_block_data(lsm6dsox_address, 0x28, 2)
    accel_y = bus.read_i2c_block_data(lsm6dsox_address, 0x2A, 2)
    accel_z = bus.read_i2c_block_data(lsm6dsox_address, 0x2C, 2)
    
    # Read gyroscope data (16-bit signed values)
    gyro_x = bus.read_i2c_block_data(lsm6dsox_address, 0x22, 2)
    gyro_y = bus.read_i2c_block_data(lsm6dsox_address, 0x24, 2)
    gyro_z = bus.read_i2c_block_data(lsm6dsox_address, 0x26, 2)
    
    # Convert raw data to values (you may need to adjust the scaling factors)
    accel_x = (accel_x[1] << 8 | accel_x[0]) / 16384.0  # 2g range
    accel_y = (accel_y[1] << 8 | accel_y[0]) / 16384.0  # 2g range
    accel_z = (accel_z[1] << 8 | accel_z[0]) / 16384.0  # 2g range
    
    gyro_x = (gyro_x[1] << 8 | gyro_x[0]) / 16.4  # 2000 dps range
    gyro_y = (gyro_y[1] << 8 | gyro_y[0]) / 16.4  # 2000 dps range
    gyro_z = (gyro_z[1] << 8 | gyro_z[0]) / 16.4  # 2000 dps range
    
    # Print the sensor data
    print(f"Accelerometer (g): X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f}")
    print(f"Gyroscope (dps): X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}")
    
    time.sleep(1)  # Wait for 1 second before reading again
