import smbus2
import time
import math

# MPU-6050 Registers
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# I2C setup
bus = smbus2.SMBus(1)  # Use I2C bus 1
DEVICE_ADDRESS = 0x68   # MPU-6050 default I2C address

# --- MPU-6050 Initialization ---
# The sensor starts in sleep mode, so we need to wake it up.
bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0)

def read_raw_data(addr):
    """
    Reads two bytes from the given address (high byte first)
    and combines them into a single 16-bit value.
    This is necessary because the sensor data for each axis is 16 bits long.
    """
    high = bus.read_byte_data(DEVICE_ADDRESS, addr)
    low = bus.read_byte_data(DEVICE_ADDRESS, addr + 1)
    
    # Combine the two bytes into a 16-bit value
    value = (high << 8) | low
    
    # Convert to signed value (from two's complement)
    if value >= 0x8000:
        return -((65535 - value) + 1)
    else:
        return value

print("Reading data from MPU-6050...")

try:
    while True:
        # --- Read Accelerometer Data ---
        accel_x = read_raw_data(ACCEL_XOUT_H)
        accel_y = read_raw_data(ACCEL_YOUT_H)
        accel_z = read_raw_data(ACCEL_ZOUT_H)
        
        # --- Read Gyroscope Data ---
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        
        # --- Scale the Raw Data ---
        # The raw values are converted to meaningful units (g's for accelerometer, degrees/sec for gyroscope)
        # by dividing by the sensitivity scale factor.
        # Default settings: Accel: +/- 2g (16384 LSB/g), Gyro: +/- 250 deg/s (131 LSB/deg/s)
        Ax = accel_x / 16384.0
        Ay = accel_y / 16384.0
        Az = accel_z / 16384.0
        
        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0
        
        # --- Print the Scaled Data ---
        print(f"Accel X: {Ax:.2f} g | Accel Y: {Ay:.2f} g | Accel Z: {Az:.2f} g")
        print(f"Gyro  X: {Gx:.2f} °/s | Gyro  Y: {Gy:.2f} °/s | Gyro  Z: {Gz:.2f} °/s")
        print("------------------------------------------------------------------")
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nMeasurement stopped by user.")
except Exception as e:
    print(f"An error occurred: {e}")