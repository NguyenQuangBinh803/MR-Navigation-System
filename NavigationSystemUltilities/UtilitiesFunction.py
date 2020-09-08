__author__ = 'EdwardAshenbert'
import cv2
import numpy as np
from datetime import datetime

refPt = []
cropping = False

# Meter
total_x = 0
total_y = 0

total_x_pixel = 0
total_y_pixel = 0

path = []


prev_front_left_angle = 0
prev_front_right_angle = 0
prev_rear_left_angle = 0
prev_rear_right_angle = 0

imu_yaw_base = 0
current_point_x = 0
current_point_y = 0

def rounding_angle(angle):
    '''
    Rounding angle make sure it between [-90, 90]
    '''
    # Cover exception returned by atan2 function

    if angle < -210:
        angle += 360
    if angle > 210:
        angle -= 360

    angle = int(angle)

    # Cover min/max
    if angle > 90:
        angle = 90
    elif angle < -90:
        angle = -90

    return angle


def display_point(image, x, y):
    cv2.circle(image, (x, y), 0, (0,255,0), -1)


def display_vehicle(vehicle, colorred_map, target=False):
    if target:
        cv2.line(colorred_map, (int(vehicle.center.x), int(vehicle.center.y)),
                 (int(vehicle.front_center_target.x), int(vehicle.front_center_target.y)),
                 (255, 255, 0), 1)
        cv2.line(colorred_map, (int(vehicle.front_left_target.x), int(vehicle.front_left_target.y)),
                 (int(vehicle.rear_left_target.x), int(vehicle.rear_left_target.y)),
                 (255, 0, 0), 1)
        cv2.line(colorred_map, (int(vehicle.front_right_target.x), int(vehicle.front_right_target.y)),
                 (int(vehicle.rear_right_target.x), int(vehicle.rear_right_target.y)),
                 (255, 0, 0), 1)
        cv2.line(colorred_map, (int(vehicle.front_left_target.x), int(vehicle.front_left_target.y)),
                 (int(vehicle.front_right_target.x), int(vehicle.front_right_target.y)),
                 (255, 0, 0), 1)
        cv2.line(colorred_map, (int(vehicle.rear_left_target.x), int(vehicle.rear_left_target.y)),
                 (int(vehicle.rear_right_target.x), int(vehicle.rear_right_target.y)),
                 (255, 0, 0), 1)
    else:
        cv2.line(colorred_map, (int(vehicle.center.x), int(vehicle.center.y)),
                 (int(vehicle.front_center.x), int(vehicle.front_center.y)),
                 (255, 255, 0), 1)
        cv2.line(colorred_map, (int(vehicle.front_left.x), int(vehicle.front_left.y)),
                 (int(vehicle.rear_left.x), int(vehicle.rear_left.y)),
                 (0, 255, 0), 1)
        cv2.line(colorred_map, (int(vehicle.front_right.x), int(vehicle.front_right.y)),
                 (int(vehicle.rear_right.x), int(vehicle.rear_right.y)),
                 (0, 255, 0), 1)
        cv2.line(colorred_map, (int(vehicle.front_left.x), int(vehicle.front_left.y)),
                 (int(vehicle.front_right.x), int(vehicle.front_right.y)),
                 (0, 255, 0), 1)
        cv2.line(colorred_map, (int(vehicle.rear_left.x), int(vehicle.rear_left.y)),
                 (int(vehicle.rear_right.x), int(vehicle.rear_right.y)),
                 (0, 255, 0), 1)


# def click_and_crop(event, x, y, flags, param):
#     global refPt, cropping
#
#     if event == cv2.EVENT_LBUTTONDOWN:
#         cropping = True
#
#     elif event == cv2.EVENT_LBUTTONUP:
#         refPt.append((x, y))
#         cropping = False
#
# cv2.namedWindow("map")
# cv2.setMouseCallback("map", click_and_crop)