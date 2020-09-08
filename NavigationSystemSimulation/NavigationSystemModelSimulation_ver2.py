__author__ = 'EdwardAshenbert'
__status__ = "20200907_1412 This is Navigation Obstacle avoidance"

import threading
import re

from NavigationSystem.AStar_5 import *
from NavigationSystem.PVector import PVector
from NavigationSystem.UtilitiesFunction import *
from NavigationSystem.UtilitiesMacroAndConstant import *
from NavigationSystem.Vehicle import Vehicle

import NavigationSystem.NavigationSystemShareMemory as navigation_system_share_memory

def get_local_cost_map(point, global_map, range):
    h, w = global_map.shape[:2]
    if point[1] - range < 0:
        point[1] = range
    elif point[1] + range > h:
        point[1] = h - range
    if point[0] - range < 0:
        point[0] = range
    elif point[0] + range > w:
        point[0] = w - range
    local_map = global_map[point[1] - range: point[1] + range, point[0] - range: point[0] + range]
    return local_map


OS_PATH = ""
DATE_TIME = "AStar_testresult/" + datetime.now().strftime("%d%m%Y_%H%M%S")


class NavigationSystemModel():

    def __init__(self, map_file):
        cv2.startWindowThread()
        self.astar = Astar(map_file)
        self.obstacle_map = cv2.imread("NavigationMapDebugImages/astar_obstacle_map_20200907_141802.png", 0)

        # self.astar_computation = Astar(OS_PATH + map_file)
        self.map_name = "client_map"
        self.path = []
        self.colorred_map = self.astar.display_map.copy()
        self.colorred_map = cv2.cvtColor(self.colorred_map, cv2.COLOR_GRAY2BGR)
        self.goal = []
        self.current_point_x = 0
        self.current_point_y = 0
        self.initial_pose_yaw = math.pi / 2
        self.init_vehicle_state()
        self.collision_detect = False
        self.current_index = 0

        self.instant_astar = Astar(map_file)
        threading.Thread(target=self.define_instant_path).start()

    def define_instant_path(self):
        while True:
            if self.collision_detect:
                instant_path = self.instant_astar.define_path()
                self.path[self.current_index:self.collision_position] = instant_path
                self.collision_detect = False
            else:
                time.sleep(0.001)

    def init_vehicle_state(self):

        for line in open(OS_PATH + "initial_pose"):
            string = re.search("(.*),(.*),(.*)", line)
            if string:
                self.current_point_x = int(string.group(1))
                self.current_point_y = int(string.group(2))
                self.initial_pose_yaw = int(string.group(3)) * math.pi / 180

        navigation_system_share_memory.current_global_x = self.current_point_x
        navigation_system_share_memory.current_global_y = self.current_point_y

        print('Initial pose complete: ' + str(self.current_point_x) + str(self.current_point_y) + str(
            self.initial_pose_yaw))
        if SHOW_IMAGE:
            cv2.imshow(self.map_name, self.colorred_map)
        time.sleep(0.5)

    def process(self, goal):

        vehicle = Vehicle(self.current_point_x, self.current_point_y, MR_WIDTH / 2, self.initial_pose_yaw)
        displaying_vehicle = Vehicle(self.current_point_x, self.current_point_y, MR_WIDTH / 2, self.initial_pose_yaw)

        # State 2 ======================= Set start and goal period =======================
        print("Set start at current point ", self.current_point_y, self.current_point_x)
        self.astar.set_start((int(self.current_point_y), int(self.current_point_x)))

        # self.astar.transform_to_global_map()
        if SHOW_IMAGE:
            cv2.imshow(self.map_name, self.astar.display_map)
        print('Waiting for goal from server ... ')

        f = open(OS_PATH + DATE_TIME + ASTAR_TEST_RESULT + ".txt", "w")
        while True:
            if goal:
                print('Defining self.path')
                start = time.time()
                self.astar.set_goal((goal[1], goal[0]))
                print("Set goal at point ", goal[1], goal[0])
                self.path = self.astar.define_path()
                for point in self.path:
                    cv2.circle(self.colorred_map, (point[1], point[0]), radius=0, color=(0, 0, 255), thickness=-1)
                print(self.path)
                print(time.time() - start)
                self.goal = []
                break
        if SHOW_IMAGE:
            cv2.imshow(self.map_name, self.colorred_map)

        # State 3 ======================= Set control for robot with available self.path =======================
        if self.path:
            orientation_head = PVector(self.path[1][1], self.path[1][0])
            orientation_tail = PVector(self.path[0][1], self.path[0][0])
            orientation = PVector.sub2Vectors(orientation_head, orientation_tail)
            path_orientation = orientation.heading()
        # display_vehicle(vehicle, colorred_map)

        # =====================================================================================================
        print('Get ready for 4 seconds')
        # time.sleep(4)
        cv2.waitKey(0)

        if self.path:
            for index, point in enumerate(self.path):
                if index != 0:
                    process_start = time.time()
                    self.current_index = index
                    # ============================ Change follow to follow vector ============================
                    current_vector = vehicle.center
                    path_vector = PVector(point[1], point[0])
                    dislocation_vector = PVector.sub2Vectors(path_vector, current_vector)

                    # Only allow to move forward
                    if dislocation_vector.mag() == 0 or abs(
                            dislocation_vector.heading() - vehicle.angle) * 180 / math.pi > VEHICLE_ANGLE_TOLERANCE:
                        continue
                    vehicle.follow_vector(dislocation_vector)

                    front_left_angle, front_right_angle, rear_left_angle, rear_right_angle = vehicle.get_next_angle()
                    front_left_speed, front_right_speed, rear_left_speed, rear_right_speed = vehicle.get_next_speed()

                    front_left_angle = rounding_angle(front_left_angle)
                    front_right_angle = rounding_angle(front_right_angle)
                    rear_left_angle = rounding_angle(rear_left_angle)
                    rear_right_angle = rounding_angle(rear_right_angle)

                    # print(front_left_angle, front_right_angle, rear_left_angle, rear_right_angle, front_left_speed,
                    #       front_right_speed, rear_left_speed, rear_right_speed)
                    f.write("[Angle]" + str(front_left_angle) + DELIMITER + str(front_right_angle) + DELIMITER + str(
                        rear_left_angle) + DELIMITER + str(rear_right_angle) + "\n")
                    f.write("[Speed]" + str(front_left_speed) + DELIMITER + str(front_right_speed) + DELIMITER + str(
                        rear_left_speed) + DELIMITER + str(rear_right_speed) + "\n")

                    display_vehicle(vehicle, self.colorred_map)

                    navigation_system_share_memory.current_global_x = vehicle.center.x
                    navigation_system_share_memory.current_global_y = vehicle.center.y
                    self.current_point_x = int(vehicle.center.x)
                    self.current_point_y = int(vehicle.center.y)
                    print("[current_position]", self.current_point_x, self.current_point_y)

                    # Local cost map anallysis ===========================================================================
                    if len(self.path[index:-1]) > 50:
                        forward_sense = 50
                    else:
                        forward_sense = len(self.path[index:-1])

                    cv2.circle(self.colorred_map, (self.path[index + forward_sense][1], self.path[index + forward_sense][0]), 2,
                               (255, 0, 0), -1)
                    obstacle_map = self.obstacle_map.copy()
                    local_cost_map = get_local_cost_map([self.current_point_x, self.current_point_y], obstacle_map, 50)

                    self.instant_astar.transform_to_global_map(PVector(int(self.current_point_x), int(self.current_point_y)), local_cost_map)
                    if SHOW_IMAGE:
                        cv2.imshow("local_cost_map", local_cost_map)
                    display_vehicle(vehicle, obstacle_map)

                    if self.instant_astar.map[self.path[index + forward_sense][0], self.path[index + forward_sense][1]] == 1:
                        print("[collision_detect]", self.path[index + forward_sense][1], self.path[index + forward_sense][0])
                        collision_index = index + forward_sense
                        self.instant_astar.set_start((self.current_point_y, self.current_point_x))
                        while (self.instant_astar.map[self.path[collision_index][0], self.path[collision_index][1]] != 0):
                            collision_index += 1
                        self.collision_position = collision_index
                        self.instant_astar.set_goal((self.path[collision_index][0], self.path[collision_index][1]))
                        self.collision_detect = True
                        cv2.circle(self.colorred_map, (self.path[collision_index][1], self.path[collision_index][0]), 2, (0, 0, 255), -1)

                    # if SHOW_IMAGE:
                    #     cv2.imshow("local_cost_map", self.instant_astar.display_map)
                    print("[navigation_share_memory]", navigation_system_share_memory.current_global_x, navigation_system_share_memory.current_global_y)
                    computing_time = time.time() - process_start
                    f.write("[computing_time]" + str(computing_time) + "\n")
                    # if computing_time < 0.15:
                    #     time.sleep(0.15 - computing_time)
                    f.write("[sampling_time]" + str(time.time() - process_start) + "\n")
                cv2.waitKey(0)

                if SHOW_IMAGE:
                    cv2.imshow(self.map_name, self.colorred_map)

            print("[done]")
            f.close()
        cv2.imwrite(DATE_TIME + ASTAR_TEST_RESULT + ".png", self.colorred_map)


def publish_point():
    while (1):
        print(1)
        time.sleep(0.05)


if __name__ == "__main__":
    navigation_system_model = NavigationSystemModel("map_10.pgm")
    navigation_system_model.process([200, 36])
