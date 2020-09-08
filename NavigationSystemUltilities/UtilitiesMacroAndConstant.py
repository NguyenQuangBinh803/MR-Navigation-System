__author__ = 'Edward J. C. Ashenbert'

import math
from datetime import datetime
# Vehicle sufficient macros
VEL_SCALE = 9
MR_HEIGHT = 30
MR_WIDTH = 48
MR_ANGLE = math.atan(MR_HEIGHT / MR_WIDTH)
MR_DISTANCE_FROM_CENTER = math.sqrt((MR_HEIGHT / 2) * (MR_HEIGHT / 2) + (MR_WIDTH / 2) * (MR_WIDTH / 2))
SCALE = 10
MR_SCALE = 1
CENTER_X = 0
CENTER_Y = 0

# Initailize speed and angle value
front_left_angle = 0
front_right_angle = 0
rear_left_angle = 0
rear_right_angle = 0

front_left_speed = 0
front_right_speed = 0
rear_left_speed = 0
rear_right_speed = 0

# Serial parameters
ROBOT_BAUDRATE = 115200
ROBOT_PORT = "/dev/ttyUSB3,/dev/ttyUSB1"
IMU_BAUDRATE = 115200
IMU_PORT = "/dev/ttyUSB0"
LIDAR_PORT = "/dev/ttyUSB3"

# Serial command format
DELIMITER = ','
ANGLE_COMMAND = 'q'
SPEED_COMMAND = 'n'
EOF = '\n'

CMD_VEL_SCALE = 200

NAV_DEBUG = True
START_FRAME = "A5"
END_FRAME = "E5"
LENGTH_OF_DATA_RECEIVE = 6

ENCODER_TO_METER = 21730
DISTANCE_THRESH = 21730

SPIN_ANGLE_STEP = 1
MAX_ENCODER_PULSE = 4294967296

# cm/pixel
MAP_RESOLUTION = 2

ASTAR_TEST_RESULT = "_ASTAR_TEST_RESULT"
SIMULATION_RESULT = "_ASTAR_SIMULATION_RESULT"

# cm/pixel
DEBUG_MODE = False
TARGET_RANGE = 20

DATE_TIME = "AStar_testresult/" + datetime.now().strftime("%d%m%Y_%H%M%S")


# NAV PARAMETERS

ENCODER_INIT_THRESHOLD = 2000
DISLOCATION_ORIENTATION_THRESH = math.pi / 3
VEHICLE_ANGLE_TOLERANCE = 100
SHOW_IMAGE = True
SYSTEM_SAMPLING_TIME = 0.15
FORWARD_RANGE = 50
OS_PATH = "/home/alpha64/Desktop/MR-Navigation-System-ROS/NavigationSystem/NavigationSystem/NavigationSystem/"
