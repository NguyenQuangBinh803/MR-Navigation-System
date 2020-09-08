__author__ = 'Edward J. C. Ashenbert'

import sys
import threading
import time
import serial
from NavigationSystem.UtilitiesMacroAndConstant import *
import datetime
import numpy as np
class Robot:
    def __init__(self, port, baud_rate):
        try:
            print("Opening serial port: %s..." % port + ".")
            self.robot_serial = serial.Serial(port, baud_rate, timeout=None)
            time.sleep(1)
            self.robot_serial.reset_input_buffer()
            threading.Thread(target=self.flush_data_serial).start()
            self.root_node = "Some thing"
            self.prev_distance_front_left = 0
            self.prev_distance_front_right = 0
            self.prev_distance_rear_left = 0
            self.prev_distance_rear_right = 0
            
        except serial.serialutil.SerialException:
            print("Serial not found at port " + port + ".")
            sys.exit(0)

    def flush_data_serial(self):
        self.robot_serial.read(self.robot_serial.in_waiting)
        time.sleep(0.3)

    def write_command(self, cmd):
        cmd = str(cmd)
        print("\t" + str(datetime.datetime.now().time()) + " --- " + "[[  ", cmd, "  ]]")
        self.robot_serial.write(bytes(cmd, 'utf-8'))

    def convert_string_to_int32(self, strData):
        return (np.int32(strData[0]) << 24) + (np.int32(strData[1]) << 16) + (np.int32(strData[2]) << 8) + (
            np.int32(strData[3]))

    def read_command(self):
        '''
        request robot current position
        '''
        self.write_command("5;")
        bytesRead = self.robot_serial.inWaiting()
        return self.robot_serial.read(bytesRead)

    def reset_dislocation_data(self):
        '''
        Everytime there's a spin behaviour we need to reset encoders for
        :return:
        '''
        raw_data = str(self.read_command(), 'utf-8').split()
        # self.write_command("5;")
        # Check length of receive buffer with start and end frame
        if (len(raw_data) == LENGTH_OF_DATA_RECEIVE and raw_data[0] == START_FRAME and raw_data[-1] == END_FRAME):
            receive_data = raw_data[1:-1]
        else:
            return [0,0,0,0]

        self.prev_distance_front_left = self.convert_string_to_int32(receive_data[1:5])
        self.prev_distance_front_right = self.convert_string_to_int32(receive_data[6:10])
        self.prev_distance_rear_left = self.convert_string_to_int32(receive_data[11:15])
        self.prev_distance_rear_right = self.convert_string_to_int32(receive_data[16:])

        self.prev_distance_front_left = 0
        self.prev_distance_front_right = 0
        self.prev_distance_rear_left = 0
        self.prev_distance_rear_right = 0

    def get_dislocation_data(self):
        '''
        Get dislocation of 4 wheel apparently

        :return: [distance_front_left, distance_front_right, distance_rear_left, distance_rear_right]
        '''
        raw_data = str(self.read_command(), 'utf-8').split()
        # self.write_command("5;")
        # Check length of receive buffer with start and end frame
        if (len(raw_data) == LENGTH_OF_DATA_RECEIVE and raw_data[0] == START_FRAME and raw_data[-1] == END_FRAME):
            receive_data = raw_data[1:-1]
        else:
            return [0,0,0,0]

        distance_front_left = self.convert_string_to_int32(receive_data[1:5])
        distance_front_right = self.convert_string_to_int32(receive_data[6:10])
        distance_rear_left = self.convert_string_to_int32(receive_data[11:15])
        distance_rear_right = self.convert_string_to_int32(receive_data[16:])

        # Mapping reverse pulse
        distance_front_right = abs(distance_front_right)
        distance_front_left = abs(distance_front_left)
        distance_rear_left = abs(distance_rear_left)
        distance_rear_right = abs(distance_rear_right)

        if NAV_DEBUG:
            print(self.root_node, "Pulse_receive_from_wheels", distance_front_left, distance_front_right,
                distance_rear_left, distance_rear_right)

        # Threshold the distance pulse return from wheels
        if abs(distance_front_left - self.prev_distance_front_left) > DISTANCE_THRESH:
            distance_front_left = self.prev_distance_front_left
        if abs(distance_front_right - self.prev_distance_front_right) > DISTANCE_THRESH:
            distance_front_right = self.prev_distance_front_right
        if abs(distance_rear_left - self.prev_distance_rear_left) > DISTANCE_THRESH:
            distance_rear_left = self.prev_distance_rear_left
        if abs(distance_rear_right - self.prev_distance_rear_right) > DISTANCE_THRESH:
            distance_rear_right = self.prev_distance_rear_right

        dislocation_front_left = distance_front_left - self.prev_distance_front_left
        dislocation_front_right = distance_front_right - self.prev_distance_front_right
        dislocation_rear_left = distance_rear_left - self.prev_distance_rear_left
        dislocation_rear_right = distance_rear_right - self.prev_distance_rear_right

        self.prev_distance_front_left = distance_front_left
        self.prev_distance_front_right = distance_front_right
        self.prev_distance_rear_left = distance_rear_left
        self.prev_distance_rear_right = distance_rear_right

        if NAV_DEBUG:
            print(self.root_node, "Dislocation_pulse", dislocation_front_left, dislocation_front_right, dislocation_rear_left, dislocation_rear_right)

        return dislocation_front_left, dislocation_front_right, dislocation_rear_left, dislocation_rear_right

    def get_dislocation(self):
        '''
        Get dislocation from robot center

        :return: [center_x, center_y]
        '''

        result = self.get_dislocation_data()
        mr_center_x = 0.0
        mr_center_y = 0.0

        if result:

            dislocation_front_left = result[0]
            dislocation_front_right = result[1]
            dislocation_rear_left = result[2]
            dislocation_rear_right = result[3]

            front_left_angle, front_right_angle, rear_left_angle, rear_right_angle = self.vehicle.get_next_angle(False)

            dislocation_front_left_x = dislocation_front_left * math.cos(front_left_angle)
            dislocation_front_left_y = dislocation_front_left * math.sin(front_left_angle)

            dislocation_front_right_x = dislocation_front_right * math.cos(front_right_angle)
            dislocation_front_right_y = dislocation_front_right * math.sin(front_right_angle)

            dislocation_rear_left_x = dislocation_rear_left * math.cos(rear_left_angle)
            dislocation_rear_left_y = dislocation_rear_left * math.sin(rear_left_angle)

            dislocation_rear_right_x = dislocation_rear_right * math.cos(rear_right_angle)
            dislocation_rear_right_y = dislocation_rear_right * math.sin(rear_right_angle)

            mr_center_x = (dislocation_front_left_x + dislocation_front_right_x + dislocation_rear_left_x + dislocation_rear_right_x) / 4 / ENCODER_TO_METER
            mr_center_y = (dislocation_front_left_y + dislocation_front_right_y + dislocation_rear_left_y + dislocation_rear_right_y) / 4 / ENCODER_TO_METER

        return mr_center_x, mr_center_y

DELIMITER = ','
ANGLE_COMMAND = 'q'
SPEED_COMMAND = 'n'
EOF = ';'

if __name__ == "__main__":
    robot = Robot("COM1", 115200)
    front_left_angle = 0
    front_right_angle = 0
    rear_left_angle = 0
    rear_right_angle = 0

    front_left_speed = 0
    front_right_speed = 0
    rear_left_speed = 0
    rear_right_speed = 0

    robot.write_command(
        ANGLE_COMMAND + DELIMITER + str(front_left_angle) + DELIMITER + str(front_right_angle) + DELIMITER + str(
            rear_left_angle) + DELIMITER + str(rear_right_angle) + EOF)
    robot.write_command(
        SPEED_COMMAND + DELIMITER + str(front_left_speed) + DELIMITER + str(front_right_speed) + DELIMITER + str(
            rear_left_speed) + DELIMITER + str(rear_right_speed) + EOF)

    print(robot.get_dislocation())
