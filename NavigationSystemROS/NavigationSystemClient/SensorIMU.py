__author__ = 'Edward J. C. Ashenbert'
import sys
import time
import serial
from math import pi

class IMU:
    def __init__(self, port, baudrate):
        try:
            # Initialization MPU9250 serial port
            self.serial = serial.Serial(port, baudrate, timeout=None)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0

            self.accX = 0.0
            self.accY = 0.0
            self.accZ = 0.0

            self.gyroX = 0.0
            self.gyroY = 0.0
            self.gyroZ = 0.0

            self.rad2pi = 180.0/pi

            self.startSignal = "OK"
            self.isReady = False

        except Exception as exp:
            sys.exit(str(exp))

    # Read data imu sensor by serial port, acceleration: m/s^2, gyroscope: rad/s
    # Output: [roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
    def readIMU(self):
        if not self.isReady:
            return
        try:
            self.serial.write('1'.encode())
            raw = str(self.serial.readline().decode('utf-8')).split(',')
            
            data = [round(float(i), 5) for i in raw]
            
            self.roll = data[0]
            self.pitch = data[1]
            self.yaw = data[2]

            self.accX = data[3]
            self.accY = data[4]
            self.accZ = data[5]

            self.gyroX = data[6]
            self.gyroY = data[7]
            self.gyroZ = data[8]

            self.magX = data[9]
            self.magY = data[10]
            self.magZ = data[11]

        except (UnicodeDecodeError, AttributeError):
            return
        except OSError as os_err:
            sys.exit(str(os_err))

    def getRPYrad(self):
        return [round(self.roll/self.rad2pi , 5), round(self.pitch/self.rad2pi , 5), round(self.yaw/self.rad2pi , 5)]
        
    
    def getRPYdeg(self):
        return [self.roll, self.pitch, self.yaw]

    def getAcc(self): #m/s^2
        return [self.accX, self.accY, self.accZ]

    def getGyro(self): #rad/s
        return [self.gyroX, self.gyroY, self.gyroZ]
    
    def getMag(self): #rad/s
        return [self.magX, self.magY, self.magZ]

    def checkIMU(self):
        return self.roll != 0   or self.pitch != 0   or self.yaw != 0  or    \
               self.accX != 0   or self.accY != 0    or self.accZ != 0 or    \
               self.gyroX != 0  or self.gyroY != 0   or self.gyroZ != 0
    
    def isIMUReady(self):
        #  while not self.isReady:
        state = str(self.serial.readline().decode('utf-8'))
        print(state)
        if(self.startSignal in state):
            print(state)
            self.isReady = True
            time.sleep(0.5)
            return True
        return False

#Instruction
portName = "COM10"
br = 115200
loopTime = 10

def runTest():
    imu = IMU(portName, br) # Init IMU port
    while not imu.isIMUReady(): # Check if IMU is ready
        pass

    while(1): # Get IMU data each 10ms
        imu.readIMU()
        if imu.checkIMU():
            print(imu.getRPYdeg(), imu.getRPYrad(), imu.getAcc(), imu.getGyro())
        time.sleep(0.001 * loopTime)

# runTest()




        