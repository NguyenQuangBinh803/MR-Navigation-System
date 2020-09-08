__author__ = "Edward J. C. Ashenbert"
__version__ = "2.0.7"
__status__ = "20200908_1037"

import threading

import NavigationSystemROS.NavigationSystemServer.NavigationSystemServerShareMemory as navigation_system_server_share_memory
from NavigationSystemUltilities.AStar_5 import *
from NavigationSystemUltilities.UtilitiesFunction import *

def click_and_crop(event, x, y, flags, param):
    global refPt, cropping

    if event == cv2.EVENT_LBUTTONDOWN:
        cropping = True

    elif event == cv2.EVENT_LBUTTONUP:
        refPt.append((x, y))
        cropping = False

cv2.startWindowThread()
cv2.namedWindow("map_server")
cv2.setMouseCallback("map_server", click_and_crop)

class NavigationSystemServerModel():

    def __init__(self):
        self.astar = Astar(
            "/home/lannt-dev8/Desktop/MR-Navigation-System-ROS/NavigationSystemUI/NavigationSystemUI/NavigationSystemUI/map_2.pgm")
        self.colorred_map = cv2.cvtColor(self.astar.display_map, cv2.COLOR_GRAY2BGR)
        self.current_point = []

    def process(self):
        global refPt

        while (1):
            if refPt:
                cv2.circle(self.colorred_map, (int(refPt[0][0]), int(refPt[0][1])), 5, (255, 0, 0), -1)
                navigation_system_server_share_memory.goal_position_x = refPt[0][0]
                navigation_system_server_share_memory.goal_position_y = refPt[0][1]
                refPt = []
            cv2.circle(self.colorred_map, (int(navigation_system_server_share_memory.current_position_x),
                                           int(navigation_system_server_share_memory.current_position_y)), 5,
                       (0, 255, 0), -1)
            cv2.imshow("map_server", self.colorred_map)
            # cv2.waitKey(1)
            if cv2.waitKey(10) == 32:
                break

def main():
    print("Test server model")
    navigation_system_server_model = NavigationSystemServerModel()
    navigation_system_server_model.process()

def check_share_memory_value():
    while True:
        print(navigation_system_server_share_memory.goal_position_x,
              navigation_system_server_share_memory.goal_position_y)

if __name__ == "__main__":
    threading.Thread(target=check_share_memory_value).start()
    main()
