import math
import re

import cv2
import numpy as np

from NavigationSystem.AStar_5 import *
from NavigationSystem.PVector import PVector
from NavigationSystem.Vehicle import Vehicle
from NavigationSystem.Robot import Robot
from NavigationSystem.SensorIMU import IMU
from NavigationSystem.UtilitiesFunction import *
from NavigationSystem.UtilitiesMacroAndConstant import *

if __name__ == "__main__":
    f = open("AStar_testresult/15082020_181037_ASTAR_TEST_RESULT.txt")
    p = re.compile(r"[-+]?\d*\.\d+|\d+")
    current_point = []
    dislocation_vector = []
    orientation = []
    path_point = []
    integer = re.compile(r'\d+')
    for line in f:
        print(line)
        string = re.search("Current_Point](.*),(.*)", line)
        if string:
            current_point.append([int(string.group(1)), int(string.group(2))])
        string_2 = re.search("Dislocation_Vector](.*),(.*)", line)
        if string_2:
            dislocation_vector.append([float(string_2.group(1)), float(string_2.group(2))])
        string_3 = re.search("Orientation_Degree](.*),(.*)", line)
        if string_3:
            orientation.append([float(string_3.group(1)), float(string_3.group(2))])
        string_4 = re.search("Path_point] (.*),(.*),", line)
        if string_4:
            path_point.append([int(string_4.group(1)), int(string_4.group(2))])

    # map = np.zeros((500, 500, 3), np.uint8)
    # for index, point in enumerate(current_point):
    #     vector_x = dislocation_vector[index][0] * math.cos(dislocation_vector[index][1])
    #     vector_y = dislocation_vector[index][0] * math.sin(dislocation_vector[index][1])
    #     print((point[0], point[1]), (int(point[0] + vector_x), int(point[1] + vector_y)))
    #     print(path_point[index][0], path_point[index][1])
    #     print(orientation[index][0], orientation[index][1], orientation[index][0] - orientation[index][1])
    #     cv2.line(map, (point[0], point[1]), (int(point[0] + vector_x), int(point[1] + vector_y)), (0, 255, 0), 1)
    #     cv2.circle(map, (int(point[0] + vector_x), int(point[1] + vector_y)), 5, (255, 0, 0), -1)
    print("Done")
    vehicle_orientation = math.pi/2
    current_point_x = 71
    current_point_y = 36
    for index, point in enumerate(current_point):
        current_vector = PVector(current_point[index][0], current_point[index][1])
        vector_x = dislocation_vector[index][0] * math.cos(dislocation_vector[index][1])
        vector_y = dislocation_vector[index][0] * math.sin(dislocation_vector[index][1])
        print(vector_x, vector_y)


    vehicle = Vehicle(current_point_x, current_point_y, MR_WIDTH / 2, vehicle_orientation)
    if path_point:
        for index, point in enumerate(path_point):
            map = np.zeros((500, 500, 3), np.uint8)
            if index != 0:
                # ============================ Change follow to follow vector ============================
                current_vector = PVector(current_point[index][0], current_point[index][1])
                vector_x = dislocation_vector[index][0] * math.cos(dislocation_vector[index][1])
                vector_y = dislocation_vector[index][0] * math.sin(dislocation_vector[index][1])
                path_vector = PVector(int(point[0] + vector_x), int(point[1] + vector_y))
                # path_vector = PVector(point[1], point[0])

                dislocation_vector_real = PVector.sub2Vectors(path_vector, current_vector)

                vehicle.follow_vector(dislocation_vector_real)

                front_left_angle, front_right_angle, rear_left_angle, rear_right_angle = vehicle.get_next_angle()
                front_left_speed, front_right_speed, rear_left_speed, rear_right_speed = vehicle.get_next_speed()


                front_left_angle = rounding_angle(front_left_angle)
                front_right_angle = rounding_angle(front_right_angle)
                rear_left_angle = rounding_angle(rear_left_angle)
                rear_right_angle = rounding_angle(rear_right_angle)

                print(front_left_angle, front_right_angle, rear_left_angle, rear_right_angle, front_left_speed,
                      front_right_speed, rear_left_speed, rear_right_speed)
                display_vehicle(vehicle, map)
                cv2.imshow("map", map)
                cv2.waitKey(0)
        print("Done")