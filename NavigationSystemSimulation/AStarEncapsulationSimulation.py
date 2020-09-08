__author__ = 'EdwardAshenbert'

from AStar.AStar_5 import *
from PVector import PVector
from UtilitiesMacroAndConstant import *
from Vehicle import Vehicle
from UtilitiesFunction import *


if __name__ == "__main__":

    # State 1 ======================= Init period =======================
    astar = Astar("map_2.pgm")
    
    colorred_map = cv2.cvtColor(astar.display_map, cv2.COLOR_GRAY2BGR)

    time.sleep(0.5)
    current_point_x = 71
    current_point_y = 36

    initial_pose_x = 71
    initial_pose_y = 36
    initial_pose_yaw = math.pi/2
    vehicle = Vehicle(initial_pose_x, initial_pose_y, MR_WIDTH / 2, initial_pose_yaw)
    displaying_vehicle = Vehicle(initial_pose_x, initial_pose_y, MR_WIDTH / 2, initial_pose_yaw)

    # State 2 ======================= Set start and goal period =======================
    astar.set_start((36,71))
    cv2.imshow('map', astar.display_map)
    while (1):
        f = open(DATE_TIME + ASTAR_TEST_RESULT + ".txt", "w")
        while (1):
            if len(refPt) == 1:
                start = time.time()
                astar.set_goal((refPt[0][1], refPt[0][0]))
                path = astar.define_path()
                for point in path:
                    cv2.circle(colorred_map, (point[1], point[0]), radius=0, color=(0, 0, 255), thickness=-1)
                print(time.time() - start)
                refPt = []
                break
            if cv2.waitKey(20) == 32:
                break
        cv2.imshow("map", colorred_map)
        cv2.waitKey(0)

        # State 3 ======================= Set control for robot with available path =======================

        orientation_head = PVector(path[1][1], path[1][0])
        orientation_tail = PVector(path[0][1], path[0][0])
        orientation = PVector.sub2Vectors(orientation_head, orientation_tail)
        path_orientation = orientation.heading()
        display_vehicle(vehicle, colorred_map)

        # =====================================================================================================
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

                    # print(front_left_angle, front_right_angle, rear_left_angle, rear_right_angle, front_left_speed,
                    #                         front_right_speed, rear_left_speed, rear_right_speed)
                    front_left_angle = rounding_angle(front_left_angle)
                    front_right_angle = rounding_angle(front_right_angle)
                    rear_left_angle = rounding_angle(rear_left_angle)
                    rear_right_angle = rounding_angle(rear_right_angle)

                    print(front_left_angle, front_right_angle, rear_left_angle, rear_right_angle, front_left_speed,
                        front_right_speed, rear_left_speed, rear_right_speed)

                    f.write("Angle_Speed_Sampling" + datetime.now().strftime("%d/%m/%Y - %H:%M:%S.%f") + "\n")
                    f.write("Angle-" + str(front_left_angle) + DELIMITER + str(front_right_angle) + DELIMITER + str(rear_left_angle) + DELIMITER + str(rear_right_angle) + "\n")
                    f.write("Speed-" + str(front_left_speed) + DELIMITER + str(front_right_speed) + DELIMITER + str(rear_left_speed) + DELIMITER + str(rear_right_speed) + "\n")
                    display_vehicle(vehicle, colorred_map)
                    display_point(colorred_map, current_point_x, current_point_y)

                cv2.imshow("map", colorred_map)
                cv2.waitKey(0)
            astar.set_start((vehicle.center.y, vehicle.center.x))
            print("Done")
        f.close()
        cv2.imwrite(DATE_TIME + ASTAR_TEST_RESULT + ".png", colorred_map)
        cv2.waitKey(0)
