import numpy as np
import PyLidar3
import cv2

OFFSET = 100
UNIT_CONVERTER = 20
SIZE = 200

class Lidar:
    def __init__(self, LIDAR_PORT):

        self.device = PyLidar3.YdLidarX4(LIDAR_PORT)
        self.laser_map = np.zeros((SIZE, SIZE), np.uint8)
        self.lidar_scanning = None

        if self.device.Connect():
            print("[lidar_diagnostic] lidar device is connected")
            self.lidar_system_state = 1
        else:
            self.lidar_system_state = 0
            print("[lidar_diagnostic] please check the connection")

    def get_device_info(self):
        if self.lidar_system_state == 1:
            print("[lidar_diagnostic] return device info")
            return self.device.GetDeviceInfo()
        else:
            print("[lidar_diagnostic] cannot get device info")

    def start_device(self):
        if self.lidar_system_state == 1:
            print("[lidar_diagnostic] start scanning")
            self.lidar_scanning = self.device.StartScanning()
        else:
            print("[lidar_diagnostic] cannot scan")

    def get_device_data(self):
        self.lidar_data = next(self.lidar_scanning)
        self.local_cost_map = self.update_laser_map(self.lidar_data, self.laser_map)

    def filter_noise_with_contour(self, image, group_size=50):
        '''

        :param image: gray image
        :return:
        '''
        cnts = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        print(len(cnts))
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            if w * h < group_size:
                cv2.rectangle(image, (x, y), (x + w, y + h), 0, -1)
            else:
                cv2.circle(image, (int((x + w) / 2), int((y + h) / 2)), int((x + w) / 2), (0, 0, 255), 1)

        image = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY_INV)[1]
        return image

    def update_laser_map(self, data, image):
        clone = image.copy()
        for point in data:
            cv2.circle(clone, (int((point[0] / UNIT_CONVERTER + OFFSET)), int((point[1] / UNIT_CONVERTER + OFFSET))), 1, 255, -1)

        clone = self.filter_noise_with_contour(clone)
        clone = cv2.flip(clone, 1)
        # clone = cv2.resize(clone, (ROBOT_COLLISION_RANGE, ROBOT_COLLISION_RANGE))
        cv2.imshow("Laser", clone)
        return clone

if __name__ == "__main__":
    lidar = Lidar("dev/ttyUSB1")
    lidar.start_device()
    lidar.get_device_data()
