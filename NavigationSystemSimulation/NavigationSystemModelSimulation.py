__author__ = 'EdwardAshenbert'

from NavigationSystem.AStar_5 import *
from NavigationSystem.PVector import PVector
from NavigationSystem.UtilitiesFunction import *
from NavigationSystem.UtilitiesMacroAndConstant import *
import NavigationSystem.NavigationSystemShareMemory as navigation_system_share_memory
from NavigationSystem.Vehicle import Vehicle

OS_PATH = "/home/lannt-dev8/Desktop/MR-Navigation-System-ROS/NavigationSystemUI/NavigationSystem/NavigationSystem/"
DATE_TIME = "AStar_testresult/" + datetime.now().strftime("%d%m%Y_%H%M%S")

class NavigationSystemModel():

    def __init__(self, map_file):
        cv2.startWindowThread()
        self.astar = Astar(OS_PATH + map_file)
        # self.astar_computation = Astar(OS_PATH + map_file)
        self.map_name = "Client_map"
        path = []
        self.colorred_map = self.astar.display_map.copy()
        self.colorred_map = cv2.cvtColor(self.colorred_map, cv2.COLOR_GRAY2BGR)
        self.goal = []
        self.current_point_x = 0
        self.current_point_y = 0
        self.initial_pose_yaw = math.pi / 2
        self.init_vehicle_state()

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
        cv2.imshow(self.map_name, self.colorred_map)
        time.sleep(0.5)

    def process(self, goal):

        vehicle = Vehicle(self.current_point_x, self.current_point_y, MR_WIDTH / 2, self.initial_pose_yaw)
        displaying_vehicle = Vehicle(self.current_point_x, self.current_point_y, MR_WIDTH / 2, self.initial_pose_yaw)

        # State 2 ======================= Set start and goal period =======================
        print("Set start at current point ", self.current_point_y, self.current_point_x)
        self.astar.set_start((int(self.current_point_y), int(self.current_point_x)))
        cv2.imshow(self.map_name, self.astar.display_map)
        print('Waiting for goal from server ... ')

        f = open(OS_PATH + DATE_TIME + ASTAR_TEST_RESULT + ".txt", "w")
        while True:
            if goal:
                print('Defining path')
                start = time.time()
                self.astar.set_goal((goal[1], goal[0]))
                print("Set goal at point ", goal[1], goal[0])
                path = self.astar.define_path()
                # for point in path:
                #     cv2.circle(self.colorred_map, (point[1], point[0]), radius=0, color=(0, 0, 255), thickness=-1)
                print(path)
                print(time.time() - start)
                self.goal = []
                break
        cv2.imshow(self.map_name, self.colorred_map)

        # State 3 ======================= Set control for robot with available path =======================

        orientation_head = PVector(path[1][1], path[1][0])
        orientation_tail = PVector(path[0][1], path[0][0])
        orientation = PVector.sub2Vectors(orientation_head, orientation_tail)
        path_orientation = orientation.heading()
        # display_vehicle(vehicle, colorred_map)

        # =====================================================================================================
        print('Get ready for 4 seconds')
        time.sleep(4)
        print(self.current_point_x, self.current_point_y)
        if path:
            for index, point in enumerate(path):
                if index != 0:
                    # ============================ Change follow to follow vector ============================
                    current_vector = vehicle.center
                    path_vector = PVector(point[1], point[0])
                    dislocation_vector = PVector.sub2Vectors(path_vector, current_vector)
                    vehicle.follow_vector(dislocation_vector)

                    front_left_angle, front_right_angle, rear_left_angle, rear_right_angle = vehicle.get_next_angle()
                    front_left_speed, front_right_speed, rear_left_speed, rear_right_speed = vehicle.get_next_speed()

                    front_left_angle = rounding_angle(front_left_angle)
                    front_right_angle = rounding_angle(front_right_angle)
                    rear_left_angle = rounding_angle(rear_left_angle)
                    rear_right_angle = rounding_angle(rear_right_angle)

                    # print(front_left_angle, front_right_angle, rear_left_angle, rear_right_angle, front_left_speed,
                    #       front_right_speed, rear_left_speed, rear_right_speed)

                    f.write("Angle_Speed_Sampling" + datetime.now().strftime("%d/%m/%Y - %H:%M:%S.%f") + "\n")
                    f.write("Angle-" + str(front_left_angle) + DELIMITER + str(front_right_angle) + DELIMITER + str(
                        rear_left_angle) + DELIMITER + str(rear_right_angle) + "\n")
                    f.write("Speed-" + str(front_left_speed) + DELIMITER + str(front_right_speed) + DELIMITER + str(
                        rear_left_speed) + DELIMITER + str(rear_right_speed) + "\n")

                    # display_vehicle(vehicle, self.colorred_map)
                    # display_point(self.colorred_map, self.current_point_x, self.current_point_y)
                    navigation_system_share_memory.current_global_x = vehicle.center.x
                    navigation_system_share_memory.current_global_y = vehicle.center.y
                    self.current_point_x = vehicle.center.x
                    self.current_point_y = vehicle.center.y

                    print(navigation_system_share_memory.current_global_x, navigation_system_share_memory.current_global_y)
                time.sleep(0.1)
                cv2.imshow(self.map_name, self.colorred_map)


            print("Done")
            f.close()
            cv2.imwrite(DATE_TIME + ASTAR_TEST_RESULT + ".png", self.colorred_map)

# def publish():
#     while(True):
#         print(current_global_y, current_global_x)

if __name__ == "__main__":
    navigation_system_model = NavigationSystemModel("map_2.pgm")
    # threading.Thread(target=publish).start()
    navigation_system_model.process([71,150])

