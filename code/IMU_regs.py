LSM9DS1_WAIT_TIME_SECS = 0.010  # 10 msecs

################################################################################
# Accelerometer and Gyro
LSM9DS1_ADDRESS_A_G = 0x6B
LSM9DS1_WHO_AM_I = 0x0F
WHO_AM_I_AG_RSP = 0x68

# LSM9DS1 Accel/Gyro (XL/G) Registers
LSM9DS1_CTRL_REG1_G = 0x10
LSM9DS1_CTRL_REG3_G = 0x12

LSM9DS1_STATUS_REG = 0x17
LSM9DS1_CTRL_REG6_XL = 0x20
LSM9DS1_CTRL_REG7_XL = 0x21

LSM9DS1_CTRL_REG8 = 0x22
LSM9DS1_CTRL_REG9 = 0x23
LSM9DS1_FIFO_CTRL = 0x2E
LSM9DS1_FIFO_SRC = 0x2F

OUT_X_L_G = 0x18  # Angular rate sensor pitch axis (X)
OUT_X_H_G = 0x19  # Angular rate sensor pitch axis (X)
OUT_Y_L_G = 0x1A  # Angular rate sensor pitch axis (Y)
OUT_Y_H_G = 0x1B  # Angular rate sensor pitch axis (Y)
OUT_Z_L_G = 0x1C  # Angular rate sensor pitch axis (Z)
OUT_Z_H_G = 0x1D  # Angular rate sensor pitch axis (Z)

OUT_X_L_XL = 0x28  # Linear acceleration sensor X-axis
OUT_X_H_XL = 0x29  # Linear acceleration sensor X-axis
OUT_Y_L_XL = 0x2A  # Linear acceleration sensor Y-axis
OUT_Y_H_XL = 0x2B  # Linear acceleration sensor Y-axis
OUT_Z_L_XL = 0x2C  # Linear acceleration sensor Z-axis
OUT_Z_H_XL = 0x2D  # Linear acceleration sensor Z-axis

# Temperature register
OUT_TEMP_L = 0x15
OUT_TEMP_H = 0x16

# Resolution of the sensors depending of the configuration (see datasheet)
SENSITIVITY_ACCELEROMETER_2 = 0.000061
SENSITIVITY_ACCELEROMETER_4 = 0.000122
SENSITIVITY_ACCELEROMETER_8 = 0.000244
SENSITIVITY_ACCELEROMETER_16 = 0.000732
SENSITIVITY_GYROSCOPE_245 = 0.00875
SENSITIVITY_GYROSCOPE_500 = 0.0175
SENSITIVITY_GYROSCOPE_2000 = 0.07

LSM9DS1_FIFO_OFF = 0
LSM9DS1_FIFO_THS = 1
################################################################################

################################################################################
# Magnetometer
LSM9DS1_ADDRESS_M = 0x1E
LSM9DS1_WHO_AM_I = 0x0F
WHO_AM_I_M_RSP = 0x3D

# LSM9DS1 Magneto Registers
LSM9DS1_CTRL_REG1_M = 0x20
LSM9DS1_CTRL_REG2_M = 0x21
LSM9DS1_CTRL_REG3_M = 0x22
LSM9DS1_CTRL_REG4_M = 0x23
LSM9DS1_STATUS_REG_M = 0x27
LSM9DS1_OUT_X_L_M = 0x28

OFFSET_X_REG_L_M = 0x05
OFFSET_X_REG_H_M = 0x06
OFFSET_Y_REG_L_M = 0x07
OFFSET_Y_REG_H_M = 0x08
OFFSET_Z_REG_L_M = 0x09
OFFSET_Z_REG_H_M = 0x0A

OUT_X_L_M = 0x28  # Magnetometer X-axis data output
OUT_X_H_M = 0x29  # Magnetometer X-axis data output
OUT_Y_L_M = 0x2A  # Magnetometer Y-axis data output
OUT_Y_H_M = 0x2B  # Magnetometer Y-axis data output
OUT_Z_L_M = 0x2C  # Magnetometer Z-axis data output
OUT_Z_H_M = 0x2D  # Magnetometer Z-axis data output

SENSITIVITY_MAGNETOMETER_4 = 0.00014
SENSITIVITY_MAGNETOMETER_8 = 0.00029
SENSITIVITY_MAGNETOMETER_12 = 0.00043
SENSITIVITY_MAGNETOMETER_16 = 0.00058
################################################################################
