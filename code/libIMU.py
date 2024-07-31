# from  import *
import IMU_regs
import time
import sys
import numpy as np

# A LSM9DS1 class with some methods
class IMU:
    # init method or constructor
    def __init__(self, debug_name, channel, address, who_am_i, gpio_handler):
        self.debug_name = debug_name
        self.channel = channel
        self.address = address
        self.who_am_i = who_am_i

        self.gpio_handler = gpio_handler
        self.handle = None

        self.temperature = None

        self.yaw = None
        self.pitch = None
        self.roll = None

    """
       Debug function that prints the current connected LSM9DS1 (Accelerometer, Gyroscope and Magnetometer) sensor
       :param self: reference to the current instance of the class
       :return: None
    """

    def debug_connection_message(self):
        print(self.debug_name + " connected at --> " + str(hex(self.address)))
        time.sleep(0.1)


class AccelGyro(IMU):
    # """
    #    Function that initializes the current connected LSM9DS1 (Accelerometer, Gyroscope) sensor
    #    :param self: reference to the current instance of the class
    #    :return: sensor handle, sensor current scale
    # """
    def __init__(self, name, channel, address, who_am_i, gpio_handler):
        IMU.__init__(self, name, channel, address, who_am_i, gpio_handler)

        self.accel_sensitivity = None
        self.ax = None
        self.ay = None
        self.az = None


        self.gyro_sensitivity = None
        self.gx = None
        self.gy = None
        self.gz = None

        self.gBiasRaw = [0, 0, 0]
        self.gBias = [0, 0, 0]
        self.aBiasRaw = [0, 0, 0]
        self.aBias = [0, 0, 0]
        self.aRes = 1.0  # Placeholder value, update with actual resolution
        self._autoCalc = False

        self.continuousMode = None

        try:
            self.handle = self.gpio_handler.i2c_open(self.channel, self.address)  # open i2c bus
            response = self.gpio_handler.i2c_read_byte_data(self.handle, IMU_regs.LSM9DS1_WHO_AM_I)  # Who Am I command
            if hex(response) == hex(self.who_am_i):
                self.debug_connection_message()
                self.gpio_handler.i2c_write_byte(self.handle, IMU_regs.LSM9DS1_CTRL_REG8)  # send reset command
                time.sleep(IMU_regs.LSM9DS1_WAIT_TIME_SECS)  # 10 mseg
                # Init config
                self.change_REG1_Gyro(ODR=952, scale=2000, BW=0)
                self.change_REG3_Gyro(PowerMode='default', HighpassFilter='disabled', HighpassFilterFrequency=0)
                self.change_REG6_Accel(ODR=952, scale=16, BW=0,
                                       AntiAliasingFilter=408)
                self.change_CTRL_REG6_XL(frequency=952, scale=16, bandwidth=0, anti_aliasing_filter=408)
                self.change_REG7_Accel(HighResMode='disabled', DigitalFilter=0, FilterData='default',
                                       HighpassFilter='disabled')
                time.sleep(IMU_regs.LSM9DS1_WAIT_TIME_SECS)  # 10 mseg
            else:
                print("Error, Incorrect reading !!")
                return -1
        except TypeError:
            pass


    """
        Function that closes the current connected LSM9DS1 (Accelerometer, Gyroscope) sensor
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :return: None
    """

    def stop(self):
        response = self.gpio_handler.i2c_read_byte_data(self.handle, IMU_regs.LSM9DS1_WHO_AM_I)  # Who Am I command
        if hex(response) == hex(self.who_am_i):
            self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG1_G, 0x00)
            self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG6_XL, 0x00)
            self.gpio_handler.i2c_close(self.handle)  # close i2c bus
        else:
            print("Error, Incorrect reading !!")
            return -1

    """
        Function to changes the Register 1 of the Gyroscope 
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :param ODR: Gyroscope output data rate and power mode selection
        :param scale: Gyroscope full-scale selection
        :param BW: Gyroscope bandwidth selection
        :return: current Gyroscope sensitivity
    """
    def change_REG1_Gyro(self, ODR, scale, BW):
        ODR_map = {
            0: 0b000,
            14.9: 0b001,
            59.5: 0b010,
            119: 0b011,
            238: 0b100,
            476: 0b101,
            952: 0b110
        }
        
        scale_map = {
            245: (0b00, IMU_regs.SENSITIVITY_GYROSCOPE_245),
            500: (0b01, IMU_regs.SENSITIVITY_GYROSCOPE_500),
            2000: (0b11, IMU_regs.SENSITIVITY_GYROSCOPE_2000)
        }
        
        BW_map = {
            0: 0b00,
            1: 0b01,
            2: 0b10,
            3: 0b11
        }

        ODR_G = ODR_map.get(ODR, 0b011)
        if ODR_G == 0b011 and ODR not in ODR_map:
            print('No match found !! The sample rate in Gyroscope must be 0 (Power-down), 14.9, 59.5, 119, 238, 476, or 952. Default value is: 119')
        
        FS_G, self.gyro_sensitivity = scale_map.get(scale, (0b00, IMU_regs.SENSITIVITY_GYROSCOPE_245))
        if scale not in scale_map:
            print('No match found !! The scale in Gyroscope must be 245, 500, or 2000. Default value is: 245')
        
        BW_G = BW_map.get(BW, 0b00)
        if BW not in BW_map:
            print('No match found !! The bandwidth must be 0-3')

        command_REG_1 = (ODR_G << 5) | (FS_G << 3) | (0b0 << 2) | BW_G
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG1_G, command_REG_1)

    """
      Function to changes the Register 3 of the Gyroscope 
      :param self: reference to the current instance of the class
      :param handle: current communication handle
      :param PowerMode: Gyroscope Power mode functionality (Low power or not)
      :param HighpassFilter: enable or disable the Highpass Filter of the Gyroscope
      :param HighpassFilterFrequency: Gyroscope Highpass Filter frequency
      :return: None
    """

    def change_REG3_Gyro(self, PowerMode, HighpassFilter, HighpassFilterFrequency):
        PowerMode_map = {
            'default': 0b0,
            'Low-power': 0b1
        }

        HighpassFilter_map = {
            'disabled': 0b0,
            'enabled': 0b1
        }

        HighpassFilterFrequency_map = {
            0: 0b0000,
            1: 0b0001,
            2: 0b0010,
            3: 0b0011,
            4: 0b0100,
            5: 0b0101,
            6: 0b0111,
            7: 0b1000,
            8: 0b1001
        }

        LP_mode = PowerMode_map.get(PowerMode, 0b0)
        if PowerMode not in PowerMode_map:
            print('No match found !! The mode must be default or Low-power')

        HP_EN = HighpassFilter_map.get(HighpassFilter, 0b0)
        if HighpassFilter not in HighpassFilter_map:
            print('No match found !! The HighpassFilter must be disabled or enabled')

        HPCF_G = HighpassFilterFrequency_map.get(HighpassFilterFrequency, 0b0000)
        if HighpassFilterFrequency not in HighpassFilterFrequency_map:
            print('No match found !! The HighpassFilterFrequency must be 0-8')

        command_REG_3 = (LP_mode << 7) | (HP_EN << 6) | (0b00 << 4) | HPCF_G
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG3_G, command_REG_3)


    """
      Function to changes the Register 6 of the Accelerometer 
      :param self: reference to the current instance of the class
      :param handle: current communication handle
      :param ODR: Accelerometer output data rate and power mode selection
      :param scale: Accelerometer full-scale selection
      :param BW: Accelerometer bandwidth selection
      :param AntiAliasingFilter:  Anti-aliasing filter bandwidth selection
      :return: current Accelerometer sensitivity
    """

    def change_REG6_Accel(self, ODR, scale, BW, AntiAliasingFilter):
        ODR_map = {
            'Power-down': 0b000,
            10: 0b001,
            50: 0b010,
            119: 0b011,
            238: 0b100,
            476: 0b101,
            952: 0b110
        }

        scale_map = {
            2: (0b00, IMU_regs.SENSITIVITY_ACCELEROMETER_2),
            16: (0b01, IMU_regs.SENSITIVITY_ACCELEROMETER_16),
            4: (0b10, IMU_regs.SENSITIVITY_ACCELEROMETER_4),
            8: (0b11, IMU_regs.SENSITIVITY_ACCELEROMETER_8)
        }

        BW_map = {
            0: 0b0,
            1: 0b1
        }

        AntiAliasingFilter_map = {
            408: 0b00,
            211: 0b01,
            105: 0b10,
            50: 0b11
        }

        ODR_XL = ODR_map.get(ODR, 0b011)
        if ODR_XL == 0b011 and ODR not in ODR_map:
            print('No match found !! The sample rate must be power-down, 10, 50, 119, 238, 476 or 952')

        FS_XL, self.accel_sensitivity = scale_map.get(scale, (0b00, IMU_regs.SENSITIVITY_ACCELEROMETER_2))
        if scale not in scale_map:
            print('No match found !! The scale must be 2, 4, 8 or 16')

        BW_SCAL_ODR = BW_map.get(BW, 0b0)
        if BW not in BW_map:
            print('No match found !! The bandwidth must be 0-1')

        BW_XL = AntiAliasingFilter_map.get(AntiAliasingFilter, 0b00)
        if AntiAliasingFilter not in AntiAliasingFilter_map:
            print('No match found !! The AntiAliasingFilter must be 50, 105, 211 or 408')

        command_REG_6_XL = (ODR_XL << 5) | (FS_XL << 3) | (BW_SCAL_ODR << 2) | BW_XL
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG6_XL, command_REG_6_XL)

    def change_CTRL_REG6_XL(self, frequency, scale, bandwidth, anti_aliasing_filter):
        # Configure accelerometer sampling frequency, measurement scale, bandwidth and anti-aliasing filter
        frequency_bits = int((frequency / (952 / 15)) - 1).bit_length() - 1
        scale_bits = int(scale / 2) - 1
        bandwidth_bits = bandwidth & 0b11
        anti_aliasing_filter_bits = anti_aliasing_filter & 0b11
        # print(frequency_bits << 4 ,scale_bits << 1, bandwidth_bits << 2 , anti_aliasing_filter_bits)

        register = (frequency_bits << 5) | (scale_bits << 3) | (bandwidth_bits << 2) | anti_aliasing_filter_bits
        # print(register)
        # Write the register to the device
        # self.gpio_handler..i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG6_XL, register)

    """
      Function to changes the Register 7 of the Accelerometer 
      :param self: reference to the current instance of the class
      :param handle: current communication handle
      :param HighResMode: enable or disable the High Resolution Mode
      :param DigitalFilter: Accelerometer digital filter (high pass and low pass) cutoff frequency selection
      :param FilterData:  Filtered data selection.
      :param HighpassFilter:  High-pass filter enabled for acceleration sensor interrupt function on Interrupt.
      :return: None
    """

    def change_REG7_Accel(self, HighResMode, DigitalFilter, FilterData, HighpassFilter):
        HighResMode_map = {
            'disabled': 0b0,
            'enabled': 0b1
        }

        DigitalFilter_map = {
            0: 0b00,  # ODR/50
            1: 0b01,  # ODR/100
            2: 0b10,  # ODR/9
            3: 0b11   # ODR/400
        }

        FilterData_map = {
            'default': 0b0,
            'sent_To_Register_FIFO': 0b1
        }

        HighpassFilter_map = {
            'disabled': 0b0,
            'enabled': 0b1
        }

        HR = HighResMode_map.get(HighResMode, 0b0)
        if HighResMode not in HighResMode_map:
            print('No match found !! The Highres mode must be disabled or enabled')

        DCF = DigitalFilter_map.get(DigitalFilter, 0b00)
        if DigitalFilter not in DigitalFilter_map:
            print('No match found !! The DigitalFilter mode must be between 0-3')

        FDS = FilterData_map.get(FilterData, 0b0)
        if FilterData not in FilterData_map:
            print('No match found !! The FilterData mode must be default or sent_To_Register_FIFO')

        HPIS1 = HighpassFilter_map.get(HighpassFilter, 0b0)
        if HighpassFilter not in HighpassFilter_map:
            print('No match found !! The HighPassFilter mode must be disabled or enabled')

        command_REG_7_XL = (HR << 7) | (DCF << 5) | (0b00 << 3) | (FDS << 2) | (0b0 << 1) | HPIS1
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG7_XL, command_REG_7_XL)


    """
        Function that checks the availability of the sensor temperature data
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :return: 0 or 1 depending of the availability
    """

    def temperatureAvailable(self):
        if self.gpio_handler.i2c_read_byte_data(self.handle, IMU_regs.LSM9DS1_STATUS_REG) & 0x07:  # Temperature new data available
            return 1

        return 0

    """
      Function that reads the sensor temperature data
      :param self: reference to the current instance of the class
      :param handle: current communication handle
      :return: sensor temperature data
     """

    def readTemp(self):
        try:
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, IMU_regs.IMU_regs.OUT_TEMP_L, 2)
            temp = int.from_bytes(byteArray, byteorder=sys.byteorder, signed=True)
            time.sleep(IMU_regs.LSM9DS1_WAIT_TIME_SECS)  # 10 mseg

            offset = 25  # Per datasheet sensor outputs 0 typically @ 25 degrees centigrade
            self.temperature = (temp / 16.0) + (offset * 10.0 / 10.0)

            return self.temperature

        except TypeError:
            pass

    """
        Function that enables or disables the FIFO memory 
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :param enable: enable order (True or False)
        :return: None
    """

    def enableFIFO(self, enable):
        temp = self.gpio_handler.i2c_read_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG9)
        if enable:
            temp |= (1 << 1)
        else:
            temp &= ~(1 << 1)

        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG9, temp)

    """
        Function that sets the FIFO memory 
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :param fifoMode: current FIFO mode
        :param fifoThs: parameter specified buy the manufacturer
        :return: None
    """

    def setFIFO(self, fifoMode, fifoThs):

        # Limit threshold - 0x1F (31) is the maximum. If more than that was asked
        # limit it to the maximum.
        if fifoThs <= 0x1F:
            threshold = fifoThs
        else:
            threshold = 0x1F

        # xgWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F))
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.SM9DS1_FIFO_CTRL,
                                              ((fifoMode & 0x7) << 5) | (threshold & 0x1F))

    """
        Function that gets the FIFO memory samples
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :return: FIFO samples gathered
    """

    def getFIFOSamples(self):
        # return (xgReadByte(FIFO_SRC) & 0x3F)
        return self.gpio_handler.i2c_read_byte_data(self.handle, IMU_regs.LSM9DS1_FIFO_SRC) & 0x3F

    """
       Function that sets the continuous operating mode
       :param self: reference to the current instance of the class
       :param handle: current communication handle
       :return: None
    """

    def setContinuousMode(self):
        # pi.i2c_write_byte_data(handle, LSM9DS1_CTRL_REG9,
        #                      0x02)  # Enable FIFO (see docs https://www.st.com/resource/en/datasheet/DM00103319.pdf)
        self.enableFIFO(True)
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_FIFO_CTRL, 0xC0)  # Set continuous mode
        time.sleep(IMU_regs.LSM9DS1_WAIT_TIME_SECS)  # 10 mseg

        self.continuousMode = True

    """
    Function that sets the one-shot operating mode
    :param self: reference to the current instance of the class
    :param handle: current communication handle
    :return: None
    """

    def setOneShotMode(self):

        self.enableFIFO(False)
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_FIFO_CTRL, 0x00)  # Disable continuous mode
        time.sleep(IMU_regs.LSM9DS1_WAIT_TIME_SECS)  # 10 mseg

        self.continuousMode = False

    """
        Function that performs the Accelerometer and Gyroscope first calibration 
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :param AccelScale: current Accelerometer scale configuration
        :param GyroScale: current Gyroscope scale configuration
        :return: None
    """

    def calibrateAccelGyro(self, autoCalc):
        samples = 0
        aBiasRawTemp = [0, 0, 0]
        gBiasRawTemp = [0, 0, 0]
        
        # Turn on FIFO and set threshold to 32 samples
        self.enableFIFO(True)
        self.setFIFO(self.FIFO_THS, 0x1F)
        while samples < 0x1F:
            samples = (self.xgReadByte(self.FIFO_SRC) & 0x3F)  # Read number of stored samples
        
        for _ in range(samples):
            # Read the gyro data stored in the FIFO
            self.readGyro()
            gBiasRawTemp[0] += self.gx
            gBiasRawTemp[1] += self.gy
            gBiasRawTemp[2] += self.gz
            
            self.readAccel()
            aBiasRawTemp[0] += self.ax
            aBiasRawTemp[1] += self.ay
            aBiasRawTemp[2] += self.az - int(1.0 / self.aRes)  # Assumes sensor facing up!
        
        for ii in range(3):
            self.gBiasRaw[ii] = gBiasRawTemp[ii] // samples
            self.gBias[ii] = self.calcGyro(self.gBiasRaw[ii])
            self.aBiasRaw[ii] = aBiasRawTemp[ii] // samples
            self.aBias[ii] = self.calcAccel(self.aBiasRaw[ii])
        
        self.enableFIFO(False)
        self.setFIFO(self.FIFO_OFF, 0x00)
        
        if autoCalc:
            self._autoCalc = True

    """
       Function that checks the availability of the linear acceleration data
       :param self: reference to the current instance of the class
       :param handle: current communication handle
       :return: 0 or 1 depending of the availability
    """

    def accelerationAvailable(self):
        if self.continuousMode:
            if self.gpio_handler.i2c_read_byte_data(self.handle, 0x2F) & 63:  # Read FIFO_SRC. If any of the rightmost 8 bits have a value, there is data.
                return 1

            else:
                if self.gpio_handler.i2c_read_byte_data(self.handle, IMU_regs.LSM9DS1_STATUS_REG) & 0x01:  # Accelerometer new data available
                    return 1

        return 0

    """
       Function that reads the linear acceleration data
       :param self: reference to the current instance of the class
       :param handle: current communication handle
       :param scale: current Accelerometer scale configuration
       :return: ax, ax and az data
    """

    def readAcceleration(self):
        try:
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, IMU_regs.OUT_X_L_XL, 2)
            self.ax = int.from_bytes(byteArray, byteorder=sys.byteorder, signed=True)
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, IMU_regs.OUT_Y_L_XL, 2)
            self.ay = int.from_bytes(byteArray, byteorder=sys.byteorder, signed=True)
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, IMU_regs.OUT_Z_L_XL, 2)
            self.az = int.from_bytes(byteArray, byteorder=sys.byteorder, signed=True)
            time.sleep(IMU_regs.LSM9DS1_WAIT_TIME_SECS)  # 10 mseg

            self.ax = float(self.ax) * self.accel_sensitivity
            self.ay = float(self.ay) * self.accel_sensitivity
            self.az = float(self.az) * self.accel_sensitivity

        except TypeError:
            pass

    """
        Function that checks the availability of the angular acceleration data
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :return: 0 or 1 depending of the availability
    """

    def gyroscopeAvailable(self):
        if self.gpio_handler.i2c_read_byte_data(self.handle, IMU_regs.LSM9DS1_STATUS_REG) & 0x02:  # Gyroscope new data available
            return 1

        return 0

    """
        Function that reads the angular acceleration data
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :param scale: current Gyroscope scale configuration
        :return: gx, gx and gz data
    """

    def readGyroscope(self):
        try:
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, IMU_regs.OUT_X_L_G, 2)
            self.gx = int.from_bytes(byteArray, byteorder=sys.byteorder, signed=True)
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, IMU_regs.OUT_Y_L_G, 2)
            self.gy = int.from_bytes(byteArray, byteorder=sys.byteorder, signed=True)
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, IMU_regs.OUT_Z_L_G, 2)
            self.gz = int.from_bytes(byteArray, byteorder=sys.byteorder, signed=True)
            time.sleep(IMU_regs.LSM9DS1_WAIT_TIME_SECS)  # 10 mseg

            self.gx = float(self.gx) * self.gyro_sensitivity
            self.gy = float(self.gy) * self.gyro_sensitivity
            self.gz = float(self.gz) * self.gyro_sensitivity

        except TypeError:
            pass

    """
        Function that returns the angular and linear acceleration data
        :return: ax, ay, az, gx, gx and gz data
    """

    def readAccelGyro(self):
        self.readAcceleration()
        self.readGyroscope()

        return self.ax, self.ay, self.az, self.gx, self.gy, self.gz


class Magnetometer(IMU):
    # """
    #    Function that initializes the current connected LSM9DS1 (Magnetometer) sensor
    #    :param self: reference to the current instance of the class
    #    :return: sensor handle, sensor current scale
    # """
    def __init__(self, name, channel, address, who_am_i, gpio_handler):
        IMU.__init__(self, name, channel, address, who_am_i, gpio_handler)

        self.mag_sensitivity = None
        self.mx = None
        self.my = None
        self.mz = None
        
        self.mBiasRaw = [0, 0, 0]
        self.mBias = [0, 0, 0]

        try:
            self.handle = self.gpio_handler.i2c_open(self.channel, self.address)  # open i2c bus
            response = self.gpio_handler.i2c_read_byte_data(self.handle, IMU_regs.LSM9DS1_WHO_AM_I)  # Who Am I command
            if hex(response) == hex(self.who_am_i):
                self.debug_connection_message()
                self.gpio_handler.i2c_write_byte(self.handle, IMU_regs.LSM9DS1_CTRL_REG2_M)  # send reset command
                # Init config
                time.sleep(IMU_regs.LSM9DS1_WAIT_TIME_SECS)  # 10 mseg
                self.change_REG1_Magneto(TempCompensation='enabled', OperativeModeXY='Ultra', ODR=80,
                                         FastODR='disabled',
                                         SelfTest='disabled')  # Temperature compensation enable, Ultra performance, 40 Hz
                self.change_REG2_Magneto(scale=16, Reboot='default', Reset='default')  # 4 gauss
                self.change_REG3_Magneto(i2cInterface='enabled', PowerMode='default', SPIMode='write',
                                         OperationMode='Continuous')
                self.change_REG4_Magneto(OperativeModeZ='Ultra', EndianSelection='LSb')
                time.sleep(IMU_regs.LSM9DS1_WAIT_TIME_SECS)  # 10 mseg
            else:
                print("Error, Incorrect reading !!")
                return -1
        except TypeError:
            pass

    """
        Function that closes the current connected LSM9DS1 (Magnetometer) sensor
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :return: None
    """

    def stop(self):
        response = self.gpio_handler.i2c_read_byte_data(self.handle, IMU_regs.LSM9DS1_WHO_AM_I)  # Who Am I command
        if hex(response) == hex(self.who_am_i):
            self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG3_M, 0x03)
            self.gpio_handler.i2c_close(self.handle)  # close i2c bus
        else:
            print("Error, Incorrect reading !!")
            return -1

    """
      Function to changes the Register 1 of the Magnetometer 
      :param self: reference to the current instance of the class
      :param handle: current communication handle
      :param TempCompensation: Magnetometer Temperature compensation enable.
      :param OperativeModeXY: Magnetometer X and Y axes operative mode selection
      :param ODR:  Magnetometer output data rate selection.
      :param FastODR: FAST_ODR enables data rates higher than 80 Hz
      :param SelfTest: Magnetometer Self-test enable.
      :return: None
    """

    def change_REG1_Magneto(self, TempCompensation, OperativeModeXY, ODR, FastODR, SelfTest):
        TempCompensation_map = {
            'disabled': 0b0,
            'enabled': 0b1
        }

        OperativeModeXY_map = {
            'Low': 0b00,
            'Medium': 0b01,
            'High': 0b10,
            'Ultra': 0b11
        }

        ODR_map = {
            0.625: 0b000,
            1.25: 0b001,
            2.5: 0b010,
            5: 0b011,
            10: 0b100,
            20: 0b101,
            40: 0b110,
            80: 0b111
        }

        FastODR_map = {
            'disabled': 0b0,
            'enabled': 0b1
        }

        SelfTest_map = {
            'disabled': 0b0,
            'enabled': 0b1
        }

        TEMP_COMP = TempCompensation_map.get(TempCompensation, 0b1)
        if TempCompensation not in TempCompensation_map:
            print('No match found !! The Temperature compensation must be disabled or enabled')

        OM = OperativeModeXY_map.get(OperativeModeXY, 0b01)
        if OperativeModeXY not in OperativeModeXY_map:
            print('No match found !! The operative mode must be Low, Medium, High or Ultra')

        DO = ODR_map.get(ODR, 0b100)
        if ODR not in ODR_map:
            print('No match found !! The ODR mode must be 0.625, 1.25, 2.5, 5, 10, 20, 40, or 80')

        FAST_ODR = FastODR_map.get(FastODR, 0b0)
        if FastODR not in FastODR_map:
            print('No match found !! The FastODR must be disabled or enabled')

        ST = SelfTest_map.get(SelfTest, 0b0)
        if SelfTest not in SelfTest_map:
            print('No match found !! The SelfTest must be disabled or enabled')

        command_REG_1_M = (TEMP_COMP << 7) | (OM << 5) | (DO << 2) | (FAST_ODR << 1) | ST
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG1_M, command_REG_1_M)


    """
      Function to changes the Register 2 of the Magnetometer 
      :param self: reference to the current instance of the class
      :param handle: current communication handle
      :param scale: Magnetometer full-scale configuration
      :param Reboot: Magnetometer Reboot memory content
      :param Reset:  Magnetometer configuration registers and user register reset function.
      :return: current Magnetometer sensitivity
    """

    def change_REG2_Magneto(self, scale, Reboot, Reset):
        scale_map = {
            4: (0b00, IMU_regs.SENSITIVITY_MAGNETOMETER_4),
            8: (0b01, IMU_regs.SENSITIVITY_MAGNETOMETER_8),
            12: (0b10, IMU_regs.SENSITIVITY_MAGNETOMETER_12),
            16: (0b11, IMU_regs.SENSITIVITY_MAGNETOMETER_16)
        }

        Reboot_map = {
            'default': 0b0,
            'reboot': 0b1
        }

        Reset_map = {
            'default': 0b0,
            'reset': 0b1
        }

        FS, self.mag_sensitivity = scale_map.get(scale, (0b00, IMU_regs.SENSITIVITY_MAGNETOMETER_4))
        if scale not in scale_map:
            print('No match found !! The Scale must be 4, 8, 12 or 16')

        REBOOT = Reboot_map.get(Reboot, 0b0)
        if Reboot not in Reboot_map:
            print('No match found !! The Reboot must be default or reboot')

        Soft_RST = Reset_map.get(Reset, 0b0)
        if Reset not in Reset_map:
            print('No match found !! The Soft_RST must be default or reset')

        command_REG_2_M = (0b0 << 7) | (FS << 5) | (0b0 << 4) | (REBOOT << 3) | (Soft_RST << 2) | 0b00
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG2_M, command_REG_2_M)


    """
      Function to changes the Register 3 of the Magnetometer 
      :param self: reference to the current instance of the class
      :param handle: current communication handle
      :param i2cInterface:Disable I2C interface
      :param PowerMode: Magnetometer Low-power mode configuration
      :param SPIMode:  Magnetometer SPI Serial Interface mode selection
      :param OperationMode:  Magnetometer operating mode selection
      :return: None
    """

    def change_REG3_Magneto(self, i2cInterface, PowerMode, SPIMode, OperationMode):
        i2cInterface_map = {
            'enabled': 0b0,
            'disabled': 0b1
        }

        PowerMode_map = {
            'default': 0b0,
            'Low-power': 0b1
        }

        SPIMode_map = {
            'write': 0b0,
            'read/write': 0b1
        }

        OperationMode_map = {
            'Continuous': 0b00,
            'Single': 0b01,
            'Power-down': 0b10
        }

        I2C_DISABLE = i2cInterface_map.get(i2cInterface, 0b0)
        if i2cInterface not in i2cInterface_map:
            print('No match found !! The i2cInterface must be enabled or disabled')

        LP = PowerMode_map.get(PowerMode, 0b0)
        if PowerMode not in PowerMode_map:
            print('No match found !! The PowerMode must be default or Low-power')

        SIM = SPIMode_map.get(SPIMode, 0b0)
        if SPIMode not in SPIMode_map:
            print('No match found !! The SPIMode must be write or read/write')

        MD = OperationMode_map.get(OperationMode, 0b00)
        if OperationMode not in OperationMode_map:
            print('No match found !! The OperationMode must be Continuous, Single or Power-down')

        command_REG_3_M = (I2C_DISABLE << 7) | (0b0 << 6) | (LP << 5) | (0b00 << 3) | (SIM << 2) | MD
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG3_M, command_REG_3_M)


    """
      Function to changes the Register 4 of the Magnetometer 
      :param self: reference to the current instance of the class
      :param handle: current communication handle
      :param OperativeModeZ: Magnetometer Z-axis operative mode selection
      :param EndianSelection: Magnetometer big/Little Endian data selection
      :return: None
    """

    def change_REG4_Magneto(self, OperativeModeZ, EndianSelection):
        OperativeModeZ_map = {
            'Low': 0b00,
            'Medium': 0b01,
            'High': 0b10,
            'Ultra': 0b11
        }

        EndianSelection_map = {
            'LSb': 0b0,
            'MSb': 0b1
        }

        OMZ = OperativeModeZ_map.get(OperativeModeZ, 0b01)
        if OperativeModeZ not in OperativeModeZ_map:
            print('No match found !! The operative mode must be Low, Medium, High or Ultra')

        BLE = EndianSelection_map.get(EndianSelection, 0b0)
        if EndianSelection not in EndianSelection_map:
            print('No match found !! The EndianSelection must be LSb or MSb')

        command_REG_4_M = (0b0000 << 4) | (OMZ << 2) | (BLE << 1) | 0b0
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.LSM9DS1_CTRL_REG4_M, command_REG_4_M)


    """
        Function that performs the Magnetometer first calibration 
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :param scale: current Magnetometer scale configuration
        :param loadIn: order to save the biases in the Magnetometer registers
        :return: None
    """

    def calibrateMagneto(self, loadIn):
        magMin = [0, 0, 0]
        magMax = [0, 0, 0]  # The road warrior

        for i in range(128):
            while not self.magAvailable():
                pass
            self.readMag()
            magTemp = [self.mx, self.my, self.mz]
            for j in range(3):
                if magTemp[j] > magMax[j]:
                    magMax[j] = magTemp[j]
                if magTemp[j] < magMin[j]:
                    magMin[j] = magTemp[j]

        for j in range(3):
            self.mBiasRaw[j] = (magMax[j] + magMin[j]) // 2
            self.mBias[j] = self.calcMag(self.mBiasRaw[j])
            if loadIn:
                self.magOffset(j, self.mBiasRaw[j])

    """
        Function that saves the biases in the Magnetometer registers 
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :param MagnetoScale: current Magnetometer scale configuration
        :param loadIn: order to load the biases in the Magnetometer registers
        :return: None
    """

    def MagnetoOffset(self, axis, offset):
        if axis > 2:
            print("Axes out of range !!")
        msb = 0
        lsb = 0
        msb = (offset & 0xFF00) >> 8
        lsb = offset & 0x00FF
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.OFFSET_X_REG_L_M + (2 * axis), lsb)
        self.gpio_handler.i2c_write_byte_data(self.handle, IMU_regs.OFFSET_X_REG_H_M + (2 * axis), msb)

    """
      Function that checks the availability of the magnetic field data
      :param self: reference to the current instance of the class
      :param handle: current communication handle
      :return: 0 or 1 depending of the availability
    """

    def magneticFieldAvailable(self):

        if self.gpio_handler.i2c_read_byte_data(self.handle,
                                                IMU_regs.LSM9DS1_STATUS_REG_M) & 0x08:  # Magnetometer new data available
            return 1

        return 0

    """
        Function that reads the magnetic field data
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :param scale: current Magnetometer scale configuration
        :return: mx, mx and mz data
    """

    def readMagneto(self):
        try:
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, IMU_regs.OUT_X_L_M, 2)
            self.mx = int.from_bytes(byteArray, byteorder=sys.byteorder, signed=True)
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, IMU_regs.OUT_Y_L_M, 2)
            self.my = int.from_bytes(byteArray, byteorder=sys.byteorder, signed=True)
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, IMU_regs.OUT_Z_L_M, 2)
            self.mz = int.from_bytes(byteArray, byteorder=sys.byteorder, signed=True)
            time.sleep(IMU_regs.LSM9DS1_WAIT_TIME_SECS)  # 10 mseg

            self.mx = float(self.mx) * self.mag_sensitivity
            self.my = float(self.my) * self.mag_sensitivity
            self.mz = float(self.mz) * self.mag_sensitivity

            return self.mx, self.my, self.mz

        except TypeError:
            pass

   