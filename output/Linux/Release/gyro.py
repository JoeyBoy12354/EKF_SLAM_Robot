import odroid_wiringpi as wiringpi
import time

# Initialize the I2C interface
wiringpi.wiringPiSetup()

# Define the I2C address of the LSM6DSOX sensor
lsm6dsox_address = 0x6A  # Change this to your sensor's address if different

# Configure the sensor (you may need to adjust these settings)
# Example: Set accelerometer to 2g range and gyroscope to 2000 dps
wiringpi.wiringPiI2CWriteReg8(lsm6dsox_address, 0x10, 0x40)  # CTRL1_XL register
wiringpi.wiringPiI2CWriteReg8(lsm6dsox_address, 0x11, 0x40)  # CTRL2_G register

# Read accelerometer and gyroscope data
while True:
    # Read accelerometer data (16-bit signed values)
    accel_x = wiringpi.wiringPiI2CReadReg16(lsm6dsox_address, 0x28)
    accel_y = wiringpi.wiringPiI2CReadReg16(lsm6dsox_address, 0x2A)
    accel_z = wiringpi.wiringPiI2CReadReg16(lsm6dsox_address, 0x2C)

    # Read gyroscope data (16-bit signed values)
    gyro_x = wiringpi.wiringPiI2CReadReg16(lsm6dsox_address, 0x22)
    gyro_y = wiringpi.wiringPiI2CReadReg16(lsm6dsox_address, 0x24)
    gyro_z = wiringpi.wiringPiI2CReadReg16(lsm6dsox_address, 0x26)

    # Print the sensor data
    print(f"Accelerometer (g): X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f}")
    print(f"Gyroscope (dps): X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}")

    time.sleep(1)  # Wait for 1 second before reading again
