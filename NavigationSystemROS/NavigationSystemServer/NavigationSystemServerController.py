__author__ = "Edward J. C. Ashenbert"
__version__ = "2.0.7"
__status__ = "20200908_1037"

import threading

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from transforms3d.euler import euler2quat

import NavigationSystemROS.NavigationSystemServer.NavigationSystemServerShareMemory as navigation_system_server_share_memory
from NavigationSystemROS.NavigationSystemServer.NavigationSystemServerModel import NavigationSystemServerModel

class NavigationSystemServerController(Node):

    def __init__(self):

        super().__init__('navigation_system_server')
        self.get_logger().info('The Navigation System has been initialized')

        self.publisher = self.create_publisher(Pose, 'navigation_system_server/goal_position', 10)
        self.subscription = self.create_subscription(Pose, 'navigation_system_server/position_listener',
                                                     self.position_listener, 1)
        self.current_point = []
        self.prev_goal_position_x = 0
        self.prev_goal_position_y = 0

        threading.Thread(target=self.check_goal_point).start()

    def position_listener(self, msg):
        print("[Receive_message]", msg.position)
        navigation_system_server_share_memory.current_position_x = int(msg.position.x)
        navigation_system_server_share_memory.current_position_y = int(msg.position.y)


    def check_goal_point(self):
        while True:
            if navigation_system_server_share_memory.goal_position_x != self.prev_goal_position_x or navigation_system_server_share_memory.goal_position_y != self.prev_goal_position_y:
                self.publish_point([navigation_system_server_share_memory.goal_position_x, navigation_system_server_share_memory.goal_position_y, 0])
            self.prev_goal_position_x = navigation_system_server_share_memory.goal_position_x
            self.prev_goal_position_y = navigation_system_server_share_memory.goal_position_y

    def publish_point(self, point):  # input point[0] = x, point[1] = y, point[2] = yaw
        msg = Pose()
        msg.position.x = float(point[0])
        msg.position.y = float(point[1])
        msg.position.z = 0.0
        [w, x, y, z] = euler2quat(0.0, 0.0, float(point[2]))  # return w, x, y, z
        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        self.publisher.publish(msg)

def ros_control(node):
    rclpy.spin(node)

def main():
    try:
        rclpy.init(args=None)
        navigation_system_server_model = NavigationSystemServerModel()
        navigation_system_server_controller = NavigationSystemServerController()
        threading.Thread(target=ros_control, args=(navigation_system_server_controller, )).start()
        navigation_system_server_model.process()
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
