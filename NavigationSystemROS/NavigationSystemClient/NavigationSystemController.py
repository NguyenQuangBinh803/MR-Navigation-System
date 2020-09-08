__author__ = "Edward J. C. Ashenbert"
__version__ = "2.0.7"
__status__ = "20200908_1037"

import threading

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

import NavigationSystemROS.NavigationSystemClient.NavigationSystemShareMemory as navigation_system_share_memory
from NavigationSystemROS.NavigationSystemClient.NavigationSystemModel import NavigationSystemModel
from NavigationSystemUltilities.UtilitiesFunction import *
from NavigationSystemUltilities.UtilitiesMacroAndConstant import *

global_goal_x = 0
global_goal_y = 0
goal_check = False


def worker(image):
    cv2.imshow("image", image)
    cv2.waitKey(25)


class NavigationSystemController(Node):

    def __init__(self):
        super().__init__('navigation_system')
        self.get_logger().info('The Navigation System has been initialized')
        self.publisher = self.create_publisher(Pose, 'navigation_system_server/position_listener', 10)
        self.subscription = self.create_subscription(Pose, 'navigation_system_server/goal_position', self.goal_listener,
                                                     10)
        self.prev_current_global_x = 0
        self.prev_current_global_y = 0
        threading.Thread(target=self.check_global_position).start()
        self.goal = []

    def goal_listener(self, msg):
        global global_goal_x, global_goal_y, goal_check
        self.get_logger().info('Receive goal point from server ')
        yaw = 0
        # roll, pitch, yaw = quat2euler(msg.orientation)
        # self.goal.append(msg.position.x)
        # self.goal.append(msg.position.y)
        global_goal_x = int(msg.position.x)
        global_goal_y = int(msg.position.y)
        goal_check = True
        self.get_logger().info('Receive goal point from server ' + str(msg.position.x) + str(msg.position.y) + str(yaw))

    def check_global_position(self):
        print("Initailize")
        while True:

            if navigation_system_share_memory.current_global_x != self.prev_current_global_x or \
                    navigation_system_share_memory.current_global_y != self.prev_current_global_y:
                self.publish_point(
                    [navigation_system_share_memory.current_global_x, navigation_system_share_memory.current_global_y,
                     0])

            self.prev_current_global_x = navigation_system_share_memory.current_global_x
            self.prev_current_global_y = navigation_system_share_memory.current_global_y

    def publish_point(self, point):  # input point[0] = x, point[1] = y, point[2] = yaw
        msg = Pose()
        msg.position.x = float(point[0])
        msg.position.y = float(point[1])
        msg.position.z = 0.0
        # self.get_logger().info(str(current_global_x) + str(current_global_y))
        # [w, x, y, z] = euler2quat(0.0, 0.0, float(point[2]))  # return w, x, y, z
        # msg.orientation.w = w
        # msg.orientation.x = x
        # msg.orientation.y = y
        # msg.orientation.z = z
        self.publisher.publish(msg)
        print("Publish to server:", msg.position)


def ros_control(node):
    rclpy.spin(node)


def main():
    global global_goal_x, global_goal_y, goal_check
    try:
        rclpy.init(args=None)
        navigation_system_model = NavigationSystemModel("map_2.pgm", ROBOT_PORT, ROBOT_BAUDRATE, IMU_PORT, IMU_BAUDRATE,
                                                        LIDAR_PORT)

        navigation_system_controller = NavigationSystemController()
        threading.Thread(target=ros_control, args=(navigation_system_controller,)).start()

        while (1):
            if goal_check:
                navigation_system_model.process([global_goal_x, global_goal_y])
                goal_check = False

    except KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
