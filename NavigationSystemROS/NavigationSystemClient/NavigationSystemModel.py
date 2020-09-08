__author__ = "Edward J. C. Ashenbert"
__version__ = "1.0.1"
__status__ = "20200908_1037"

import
NavigationSystemShareMemory as navigation_system_share_memory
import PyLidar3
from AStar_5 import *
from PVector import PVector
from RobotGM6020 import RobotControl
from SensorIMU import IMU
from UtilitiesFunction import *
from UtilitiesMacroAndConstant import *
from Vehicle import Vehicle

OS_PATH = "/home/alpha64/Desktop/MR-Navigation-System-ROS/NavigationSystem/NavigationSystem/NavigationSystem/"
DATE_TIME = "NavigationSystemTestResult/" + datetime.now().strftime("%Y%m%d_%H%M%S")


class NavigationSystemModel():

    def __init__(self, map_file, robot_port, robot_baudrate, imu_port, imu_baudrate, lidar_port):
        self.total_y = 0.0
        self.total_x = 0.0
        self.astar = Astar(OS_PATH + map_file)

        self.map_name = "client_map"
        self.path = []
        self.colorred_map = self.astar.display_map.copy()
        self.colorred_map = cv2.cvtColor(self.colorred_map, cv2.COLOR_GRAY2BGR)
        self.goal = []
        self.current_point_x = 0
        self.current_point_y = 0
        self.initial_pose_yaw = math.pi / 2
        self.imu_yaw_base = 0
        self.vehicle_orientation = math.pi / 2
        self.lidar_data = None
        self.logging = None
        self.path = []
        self.system_state = 0
        self.init_vehicle_state()
        self.init_robot_state(robot_port, robot_baudrate, imu_port, imu_baudrate, lidar_port)
        # Logging function

        self.vehicle = None
        self.displaying_vehicle = None

        self.initail_pos_x = 0
        self.initail_pos_y = 0

        cv2.startWindowThread()

    def init_robot_state(self, robot_port, robot_baudrate, imu_port, imu_baudrate, lidar_port):

        if robot_port:
            print("Initialize robot ......")
            self.robot = RobotControl(robot_port, robot_baudrate)
            self.robot.write_drive_command('n,0,0,0,0;')
            self.robot.write_steer_command('q,0,0,0,0;')
            print("Done initialize robot ......")
            self.system_state += 1
        else:
            print("Robot is not in use")

        if imu_port:
            print("Initailize IMU ......")
            self.imu = IMU(imu_port, imu_baudrate)
            while not self.imu.isIMUReady():
                pass
            self.imu.readIMU()
            if self.imu.checkIMU():
                self.imu_yaw_base = self.imu.getRPYdeg()[2]
            print(self.imu_yaw_base)
            print("Done initailize IMU ......")
            self.system_state += 1
        else:
            print("IMU is not in use")

        if lidar_port:
            print("Initailize Lidar ......")
            self.lidar = PyLidar3.YdLidarX4(lidar_port)
            if (self.lidar.Connect()):
                print(self.lidar.GetDeviceInfo())
                self.lidar_data = self.lidar.StartScanning()
            else:
                print("Error connecting to device please replug in device")
            print("Done initailize Lidar ......")
            self.system_state += 1
        else:
            print("Lidar is not in use")

        time.sleep(0.5)

    def rounding_angle(self, angle):
        '''
        Rounding angle make sure it between [-90, 90]
        '''
        # Cover exception returned by atan2 function

        if angle < -210:
            angle += 360
        if angle > 210:
            angle -= 360

        # angle = int(angle)

        # Cover min/max
        if angle > 90:
            angle = 90
        elif angle < -90:
            angle = -90

        return angle

    def init_vehicle_state(self):

        for line in open(OS_PATH + "initial_pose"):
            string = re.search("(.*),(.*),(.*)", line)
            if string:
                self.current_point_x = int(string.group(1))
                self.current_point_y = int(string.group(2))
                self.initial_pose_yaw = int(string.group(3)) * math.pi / 180

        navigation_system_share_memory.current_global_x = self.current_point_x
        navigation_system_share_memory.current_global_y = self.current_point_y

        self.initail_pos_x = self.current_point_x
        self.initail_pos_y = self.current_point_y

        print('Initial pose complete: ' + str(self.current_point_x) + str(self.current_point_y) + str(
            self.initial_pose_yaw))
        cv2.imshow(self.map_name, self.colorred_map)
        time.sleep(0.5)

    def path_define(self, goal):
        DATE_TIME = "NavigationSystemTestResult/" + datetime.now().strftime("%Y%m%d_%H%M%S")
        self.logging = open(OS_PATH + DATE_TIME + ASTAR_TEST_RESULT + ".txt", "w")
        print('Waiting for goal from server ... ')
        print('Defining path')
        start = time.time()
        print("Set start at current point ", self.current_point_y, self.current_point_x)
        self.astar.set_start((int(self.current_point_y), int(self.current_point_x)))
        self.astar.set_goal((goal[1], goal[0]))
        self.path = self.astar.define_path()
        for point in self.path:
            cv2.circle(self.colorred_map, (point[1], point[0]), radius=0, color=(0, 0, 255), thickness=-1)
        print(self.path)
        print(time.time() - start)

    def vehicle_orientation_correct(self):

        if self.path:
            path_orientation = PVector.sub2Vectors(PVector(self.path[5][1], self.path[5][0]),
                                                   PVector(self.path[0][1], self.path[0][0])).heading()
            if path_orientation < -math.pi / 2:
                path_orientation += 2 * math.pi

            # Check IMU data for orientation on map, spin until angle match
            self.imu.readIMU()
            self.vehicle_orientation = (self.imu.getRPYdeg()[
                                            2] - self.imu_yaw_base) * math.pi / 180 + self.initial_pose_yaw

            if abs(path_orientation - self.vehicle_orientation) > DISLOCATION_ORIENTATION_THRESH:
                sign = (path_orientation - self.vehicle_orientation) / abs(path_orientation - self.vehicle_orientation)
                dislocated_orientation = -1 * sign * (path_orientation - self.vehicle_orientation)
                self.robot.write_command("3," + str(dislocated_orientation) + ";")
                print("[dislocated_orientation]", int(abs(path_orientation - self.vehicle_orientation) * 180 / math.pi))

                while not int(abs(path_orientation - self.vehicle_orientation) * 180 / math.pi) == 0:
                    self.imu.readIMU()
                    self.vehicle_orientation = (self.imu.getRPYdeg()[
                                                    2] - self.imu_yaw_base) * math.pi / 180 + self.initial_pose_yaw
                    self.logging.write("[Vehicle_Orientation] " + str(self.vehicle_orientation) + EOF)

                # Reset encoder by changing to speed mode
                self.robot.write_command('V,;')
                self.robot.write_command('n,0,0,0,0;')
                self.robot.write_command('q,0,0,0,0;')
                self.robot.reset_dislocation_data()

                # Verify if the encoder has been reset or not
                dislocation_front_left, dislocation_front_right, _, _ = self.robot.get_dislocation_data()

                while (dislocation_front_left + dislocation_front_right) / 2 > ENCODER_INIT_THRESHOLD:
                    dislocation_mean = (dislocation_front_left + dislocation_front_right) / 2
                    self.logging.write("[Dislocation_Not_Reset] " + str(dislocation_mean) + EOF)
                    self.robot.write_command('8,;')
                    self.robot.write_command('n,0,0,0,0;')
                    self.robot.write_command('q,0,0,0,0;')
                    self.robot.reset_dislocation_data()
                    dislocation_front_left, dislocation_front_right, _, _ = self.robot.get_dislocation_data()

                dislocation_mean = (dislocation_front_left + dislocation_front_right) / 2
                self.logging.write("[Dislocation_Reset] " + str(dislocation_mean) + EOF)
                time.sleep(4)

    def vehicle_orientation_correct_japan(self):

        path_orientation = PVector.sub2Vectors(PVector(self.path[5][1], self.path[5][0]),
                                                   PVector(self.path[0][1], self.path[0][0])).heading()
        if path_orientation < -math.pi / 2:
            path_orientation += 2 * math.pi

        # Check IMU data for orientation on map, spin until angle match
        self.imu.readIMU()
        self.vehicle_orientation = (self.imu.getRPYdeg()[2] - self.imu_yaw_base) * math.pi / 180 + self.initial_pose_yaw
        sign = (path_orientation - self.vehicle_orientation) / abs(path_orientation - self.vehicle_orientation)
        self.logging.write("[Error_orientation]" + str(abs(path_orientation - self.vehicle_orientation) * 180 / math.pi) + EOF)

        if abs(path_orientation - self.vehicle_orientation) > DISLOCATION_ORIENTATION_THRESH:
            sign = (path_orientation - self.vehicle_orientation) / abs(path_orientation - self.vehicle_orientation)
            dislocated_orientation = sign * 720
            self.robot.write_command("3," + str(dislocated_orientation) + ";")
            start = time.time()
            while not int(abs(path_orientation - self.vehicle_orientation) * 180 / math.pi) == 0:
                
                self.imu.readIMU()
                self.vehicle_orientation = (self.imu.getRPYdeg()[
                                                2] - self.imu_yaw_base) * math.pi / 180 + self.initial_pose_yaw
                if self.vehicle_orientation > math.pi:
                    self.vehicle_orientation -= 2*math.pi
                self.logging.write("[Vehicle_Orientation] " + str(self.vehicle_orientation*180/math.pi)  + DELIMITER + str(path_orientation*180/math.pi) + EOF)

            
            for _ in range(5):
                self.robot.write_command('8,;')
                time.sleep(0.1)

            self.robot.write_command('n,0,0,0,0;')
            self.robot.write_command('q,0,0,0,0;')
            
            
            # Verify if the encoder has been reset or not
            # dislocation_front_left, dislocation_front_right, _, _ = self.robot.get_dislocation_data()
            for _ in range(5):
                self.robot.reset_dislocation_data()
                time.sleep(0.1)

            time.sleep(1)

    def init_vehicle_control(self):

        self.vehicle = Vehicle(self.current_point_x, self.current_point_y, MR_WIDTH / 2, self.vehicle_orientation)
        self.displaying_vehicle = Vehicle(self.current_point_x, self.current_point_y, MR_WIDTH / 2,
                                          self.vehicle_orientation)

    def path_following(self):

        print('Get ready for 2 seconds')
        for _ in range(5):
            data = self.robot.get_dislocation_data()
            time.sleep(0.1)
            print("[Flush_first_data]", data)
        time.sleep(0.5)
        index = 0

        if self.path:
            while index < len(self.path) - 1:
                index += 2
                
                if index > len(self.path) - 1:
                    index = len(self.path) - 1

                point = self.path[index]
                process_start = time.time()

                # ============================ Change follow to follow vector ============================
                current_vector = PVector(self.current_point_x, self.current_point_y)
                path_vector = PVector(point[1], point[0])
                dislocation_vector = PVector.sub2Vectors(path_vector, current_vector)

                if index == len(self.path) - 1 and dislocation_vector.mag() > 0.5:
                    index -= 2
                    self.logging.write("[time_to_close] " + str(index) + DELIMITER + str(dislocation_vector.mag()) + EOF)

                if NAV_DEBUG:
                    self.logging.write("[index] " + str(index) + EOF)
                    self.logging.write("[Path_point] " + str(point[1]) + DELIMITER + str(point[0]) + EOF)
                    self.logging.write("[Dislocation_Vector] " + str(dislocation_vector.mag()) + DELIMITER + str(
                        dislocation_vector.heading()) + EOF)

                    # Checkout orientation to the path
                    self.logging.write("[Dislocation_Angle_To_Path] " + str(
                        abs(dislocation_vector.heading() - self.vehicle_orientation) * 180 / math.pi) + EOF)
                    self.logging.write(
                        "[Orientation_Degree] " + str(
                            dislocation_vector.heading() * 180 / math.pi) + DELIMITER + str(
                            self.vehicle_orientation * 180 / math.pi) + EOF)

                # If orientation between path and vehicle is 90 degree that mean it need to change to
                # another point forward
                if self.vehicle_orientation * 180 / math.pi > 180:
                    self.vehicle_orientation -= 2 * math.pi

                # Update virtual vehicle with real orientation and init real point and target
                self.vehicle.center = current_vector
                self.vehicle.angle = self.vehicle_orientation
                self.vehicle.update()
                self.vehicle.init_real_points()

                if NAV_DEBUG:
                    self.logging.write("[Vehicle_Center] " + str(self.vehicle.center.x) + DELIMITER + str(
                        self.vehicle.center.y) + EOF)
                    self.logging.write("[Vehicle_Angle] " + str(self.vehicle.angle) + EOF)

                heading_orientation = abs(dislocation_vector.heading())
                self.logging.write("[dislocation_vector_and] " + str(dislocation_vector.heading() * 180 / math.pi) + str(self.vehicle_orientation*180/math.pi) + EOF)

                # Only allow to move forward
                if dislocation_vector.mag() == 0 or abs(
                        heading_orientation - abs(self.vehicle_orientation)) * 180 / math.pi > VEHICLE_ANGLE_TOLERANCE:
                    continue

                self.vehicle.follow_vector(dislocation_vector)

                front_left_angle, front_right_angle, rear_left_angle, rear_right_angle = self.vehicle.get_next_angle()
                front_left_speed, front_right_speed, rear_left_speed, rear_right_speed = self.vehicle.get_next_speed()

                self.logging.write(
                    "[Angle_before_rounding] " + str(front_left_angle) + DELIMITER + str(front_right_angle) + DELIMITER + str(
                        rear_left_angle) + DELIMITER + str(rear_right_angle) + EOF)

                front_left_angle = self.rounding_angle(front_left_angle)
                front_right_angle = self.rounding_angle(front_right_angle)
                rear_left_angle = self.rounding_angle(rear_left_angle)
                rear_right_angle = self.rounding_angle(rear_right_angle)

                self.logging.write(
                    "[Angle] " + str(front_left_angle) + DELIMITER + str(front_right_angle) + DELIMITER + str(
                        rear_left_angle) + DELIMITER + str(rear_right_angle) + EOF)
                self.logging.write(
                    "[Speed] " + str(front_left_speed) + DELIMITER + str(front_right_speed) + DELIMITER + str(
                        rear_left_speed) + DELIMITER + str(rear_right_speed) + EOF)

                self.robot.write_command(
                    SPEED_COMMAND + DELIMITER + str(front_left_speed) + DELIMITER + str(
                        front_right_speed) + DELIMITER + str(
                        rear_left_speed) + DELIMITER + str(rear_right_speed))

                self.robot.write_command(
                    ANGLE_COMMAND + DELIMITER + str(front_left_angle) + DELIMITER + str(
                        front_right_angle) + DELIMITER + str(
                        rear_left_angle) + DELIMITER + str(rear_right_angle))

                # State 4 ======================= Get robot dislocation =======================
                dislocation_front_left, dislocation_front_right, dislocation_rear_left, dislocation_rear_right = self.robot.get_dislocation_data()
                angle_front_left, angle_front_right, angle_rear_left, angle_rear_right = self.robot.get_angle_data()

                self.logging.write(
                    "[dislocation_pulse] " + str(dislocation_front_left) + DELIMITER + str(dislocation_front_right) + DELIMITER + str(
                        dislocation_rear_left) + DELIMITER + str(dislocation_rear_right) + EOF)
                self.logging.write(
                    "[real_angle] " + str(angle_front_left) + DELIMITER + str(angle_front_right) + DELIMITER + str(
                        angle_rear_left) + DELIMITER + str(angle_rear_right) + EOF)

                dislocation_front_left_x = dislocation_front_left * math.cos(angle_front_left * math.pi / 180)
                dislocation_front_left_y = dislocation_front_left * math.sin(angle_front_left * math.pi / 180)

                dislocation_front_right_x = dislocation_front_right * math.cos(angle_front_right * math.pi / 180)
                dislocation_front_right_y = dislocation_front_right * math.sin(angle_front_right * math.pi / 180)

                dislocation_rear_left_x = dislocation_rear_left * math.cos(angle_rear_left * math.pi / 180)
                dislocation_rear_left_y = dislocation_rear_left * math.sin(angle_rear_left * math.pi / 180)

                dislocation_rear_right_x = dislocation_rear_right * math.cos(angle_rear_right * math.pi / 180)
                dislocation_rear_right_y = dislocation_rear_right * math.sin(angle_rear_right * math.pi / 180)

                dislocation_mr_center_x = (
                        dislocation_front_left_x + dislocation_front_right_x + dislocation_rear_left_x + dislocation_rear_right_x)
                dislocation_mr_center_y = (
                        dislocation_front_left_y + dislocation_front_right_y + dislocation_rear_left_y + dislocation_rear_right_y)

                # Get alpga angle by imu
                self.imu.readIMU()
                self.vehicle_orientation = (self.imu.getRPYdeg()[
                                                2] - self.imu_yaw_base + self.initial_pose_yaw * 180 / math.pi) * math.pi / 180

                # Estimate with virtual wheel angle
                if dislocation_mr_center_x != 0:
                    dislocation_orientation = math.atan2(dislocation_mr_center_y, dislocation_mr_center_x)
                else:
                    dislocation_orientation = 0

                if NAV_DEBUG:
                    self.logging.write("[Vehicle_Orientation] " + str(self.vehicle_orientation) + EOF)
                    self.logging.write("[Dislocation_Orientation] " + str(dislocation_orientation) + EOF)

                dislocation_mr_center = math.sqrt(dislocation_mr_center_y ** 2 + dislocation_mr_center_x ** 2)

                real_dislocation_mr_center_x = dislocation_mr_center * math.cos(
                    self.vehicle_orientation + dislocation_orientation)
                real_dislocation_mr_center_y = dislocation_mr_center * math.sin(
                    self.vehicle_orientation + dislocation_orientation)

                if NAV_DEBUG:
                    self.logging.write("[Real_Dislocation] " + str(real_dislocation_mr_center_x) + DELIMITER + str(
                        real_dislocation_mr_center_y) + EOF)

                mr_center_x = real_dislocation_mr_center_x / 4 / ENCODER_TO_METER
                mr_center_y = real_dislocation_mr_center_y / 4 / ENCODER_TO_METER

                self.total_x += mr_center_x
                self.total_y += mr_center_y

                total_x_pixel = int(self.total_x / MAP_RESOLUTION * 100)
                total_y_pixel = int(self.total_y / MAP_RESOLUTION * 100)

                print("[total_pixel]", total_x_pixel, total_y_pixel, real_dislocation_mr_center_x,
                      real_dislocation_mr_center_y)

                if NAV_DEBUG:
                    self.logging.write(
                        "[Dislocation_Sampling] " + datetime.now().strftime("%d/%m/%Y_%H:%M:%S.%f") + EOF)
                    self.logging.write(
                        "[Dislocation_Meter] " + str(self.total_x) + DELIMITER + str(self.total_y) + DELIMITER + str(
                            mr_center_x) + DELIMITER + str(mr_center_y) + EOF)
                    self.logging.write(
                        "[Dislocation_Pixel] " + str(total_x_pixel) + DELIMITER + str(total_y_pixel) + EOF)

                self.current_point_x = self.initail_pos_x + total_x_pixel
                self.current_point_y = self.initail_pos_y + total_y_pixel

                print("[dislocation_distance]", self.current_point_x, self.current_point_y, mr_center_x,
                      mr_center_y)

                navigation_system_share_memory.current_global_x = self.current_point_x
                navigation_system_share_memory.current_global_y = self.current_point_y

                self.logging.write(
                    "[Current_Point]" + str(self.current_point_x) + DELIMITER + str(self.current_point_y) + EOF)

                display_point(self.colorred_map, int(self.vehicle.center.x), int(self.vehicle.center.y))
                computing_time = time.time() - process_start
                self.logging.write("[computing_time]" + str(computing_time) + "\n")

                if computing_time < 0.05:
                    time.sleep(0.05 - computing_time)

        self.robot.write_command("q,0,0,0,0;")
        self.robot.write_command("n,0,0,0,0;")

        distance_to_target = math.sqrt(
            (self.current_point_x - self.path[-1][1]) ** 2 + (
                    self.current_point_y - self.path[-1][0]) ** 2) * MAP_RESOLUTION

        display_vehicle(self.vehicle, self.colorred_map)
        cv2.imwrite(DATE_TIME + ASTAR_TEST_RESULT + ".jpeg", self.colorred_map)
        print("Write image")
        self.logging.write("[Complete]" + str(distance_to_target) + EOF)
        time.sleep(0.5)
        self.logging.close()
            # cv2.waitKey(0)

if __name__ == "__main__":
    navigation_system_model = NavigationSystemModel("map_demo_2.pgm", ROBOT_PORT, ROBOT_BAUDRATE, IMU_PORT, IMU_BAUDRATE,
                                                    None)
    navigation_system_model.init_vehicle_state()
    navigation_system_model.init_vehicle_control()
    navigation_system_model.path_define([39, 230])
    # navigation_system_model.path_define([127, 380])
    navigation_system_model.vehicle_orientation_correct_japan()
    navigation_system_model.path_following()                                                                                                                                                                                    

    # navigation_system_model.path_define([530, 550])
    # # navigation_system_model.path_define([127, 380])
    # navigation_system_model.vehicle_orientation_correct_japan()
    # navigation_system_model.path_following()

    # navigation_system_model.path_define([530, 40])
    # # navigation_system_model.path_define([127, 380])
    # navigation_system_model.vehicle_orientation_correct_japan()
    # navigation_system_model.path_following()

    # navigation_system_model.path_define([112, 239])
    navigation_system_model.path_define([39, 47])
    navigation_system_model.vehicle_orientation_correct_japan()
    navigation_system_model.path_following()
    # navigation_system_model.path_define([127, 380])
    navigation_system_model.path_define([39, 130])
    navigation_system_model.vehicle_orientation_correct_japan()
    # navigation_system_model.path_following()

    # navigation_system_model.path_define([129, 225])
    # navigation_system_model.vehicle_orientation_correct_japan()
    # navigation_system_model.path_following()

    # navigation_system_model.path_define([129, 380])
    # navigation_system_model.vehicle_orientation_correct_japan()

    time.sleep(1)

    
