__author__ = 'Edward J. C. Ashenbert'

import heapq
from MapReader import *
import time
import math

# cm/pixel
MAP_RESOLUTION = 2
DEBUG_MODE = False
TARGET_RANGE = 20


class Astar:
    def __init__(self, map_file, pgm_file=True):
        if pgm_file:
            self.safe_bounding_map = None
            self.display_map, self.map, self.map_height, self.map_width = self.read_map(map_file)
            self.original_map = self.display_map
            self.start = (0, 0)
            self.goal = (self.map_height, self.map_width)
        else:
            print(map_file.shape)
            self.map_height, self.map_width = map_file.shape[:2]
            self.display_map = map_file
            self.convert_map_to_grid(map_file)

    def safe_bouding(self, grid, distance_to_wall, safe=False):
        if safe:
            pixels = int(distance_to_wall / MAP_RESOLUTION)
            kernel = np.ones((pixels, pixels), np.uint8)
            # kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (pixels, int(pixels)))
            dilate_grid = cv2.erode(grid, kernel, iterations=1)
            if DEBUG_MODE:
                cv2.imshow('grid', grid)
                cv2.imshow('dilate_grid', dilate_grid)
            return dilate_grid
        else:
            return grid

    def convert_map_to_grid(self, display_map):
        grid = cv2.threshold(display_map, 0, 255, cv2.THRESH_BINARY)[1]
        # cv2.imshow("grid_origin", grid)
        grid = self.safe_bouding(grid, 80, True)
        # cv2.imshow("grid", grid)
        check = grid == 255
        grid[check == False] = 1
        grid[check] = 0
        self.map = grid

    def read_map(self, map_file):
        '''
        Read pgm map file
        :return:
        '''
        byteorder = '>'
        with open(map_file, 'rb') as f:
            buffer = f.read()
        try:
            header, width, height, maxval = re.search(
                b"(^P5\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
        except AttributeError:
            raise ValueError("Not a raw PGM file: '%s'" % map_file)

        display_map = np.frombuffer(buffer,
                                    dtype='u1' if int(maxval) < 256 else byteorder + 'u2',
                                    count=int(width) * int(height),
                                    offset=len(header)
                                    ).reshape((int(height), int(width)))
        (h, w) = display_map.shape[:2]
        # calculate the center of the image
        # center = (w / 2, h / 2)
        # M = cv2.getRotationMatrix2D(center, 90, 1)
        # display_map = cv2.warpAffine(display_map, M, (h, w))
        display_map = cv2.flip(display_map, -1)
        cv2.imshow("display_map", display_map)
        # Convert that map file into maze matrix 0 and 1
        grid = cv2.threshold(display_map, 100, 255, cv2.THRESH_BINARY)[1]
        grid = self.safe_bouding(grid, 80, True)
        self.safe_bounding_map = grid.copy()
        # cv2.imshow("grid", grid)
        check = grid == 255
        grid[check == False] = 1
        grid[check] = 0

        return display_map, grid, height, width

    def heuristic(self, a, b):

        # -------------Octile Heuristic----------------------------

        xVal = abs(a[0] - b[0])
        yVal = abs(a[1] - b[1])

        return max(xVal, yVal) + ((math.sqrt(2) - 1) * min(xVal, yVal))

        # return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def astar(self):
        #         neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 2), (2, 1),(2, -1),(1, -2), (2, -1), (-1, -2), (-2, -1),(-2, 1), (-1, 2)]
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        close_set = set()
        came_from = {}
        gscore = {self.start: 0}
        fscore = {self.start: self.heuristic(self.start, self.goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[self.start], self.start))
        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == self.goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                if 0 <= neighbor[0] < self.map.shape[0]:
                    if 0 <= neighbor[1] < self.map.shape[1]:
                        if self.map[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        continue

                else:
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))

        return None

    def set_goal(self, goal):
        self.goal = goal

    def set_start(self, start):
        self.start = start

    def define_path(self):
        route = self.astar()
        if route:
            route = route + [self.start]
            route = route[::-1]
            # print(route)
            return route
        else:
            print("Cannot find valid path")
            return None

    def transform_to_global_map(self, robot_current_position, robot_surrounding_state):
        '''

        :param robot_current_position: real position x,y of robot
        :param robot_surrounding_state: laser map, or sonar map
        :return:
        '''
        temp_global_map = self.display_map.copy()

        h, w = robot_surrounding_state.shape
        print("Temp global map ", temp_global_map[0:int(robot_current_position.y + h / 2), int(robot_current_position.x - w / 2): int(robot_current_position.x + w / 2)].shape)
        print("Local cost map ",robot_surrounding_state[int(h / 2 - robot_current_position.y):h, 0:w].shape)
        map_height, map_width = self.display_map.shape

        # We determine a Cartesian coordinate with four region and cover all of them in conditions
        if robot_current_position.y - h / 2 < 0:
            if robot_current_position.x - w / 2 < 0:
                temp_global_map[0:int(robot_current_position.y + h / 2), 0: int(robot_current_position.x + w / 2)] = cv2.bitwise_and(temp_global_map[0:int(robot_current_position.y + h / 2), 0: int(robot_current_position.x + w / 2)], robot_surrounding_state[int(h / 2 - robot_current_position.y):h, int(w / 2 - robot_current_position.x):w])
            elif robot_current_position.x - w / 2 > 0:
                temp_global_map[0:int(robot_current_position.y + h / 2), int(robot_current_position.x - w / 2): int(robot_current_position.x + w / 2)] = cv2.bitwise_and(temp_global_map[0:int(robot_current_position.y + h / 2), int(robot_current_position.x - w / 2): int(robot_current_position.x + w / 2)], robot_surrounding_state[int(h / 2 - robot_current_position.y):h, 0:w])

        elif robot_current_position.y + h / 2 >= map_height:
            if robot_current_position.x + w / 2 < map_width:
                temp_global_map[int(robot_current_position.y - h / 2):int(map_height), int(robot_current_position.x - w / 2): int(robot_current_position.x + w / 2)] = cv2.bitwise_and(temp_global_map[int(robot_current_position.y - h / 2):int(map_height), int(robot_current_position.x - w / 2): int(robot_current_position.x + w / 2)], robot_surrounding_state[0:int(h / 2 + map_height - robot_current_position.y), 0:w])
            elif robot_current_position.x + w / 2 > map_width:
                temp_global_map[int(robot_current_position.y - h / 2):int(map_height), int(robot_current_position.x - w / 2): int(map_width)] = cv2.bitwise_and(temp_global_map[int(robot_current_position.y - h / 2):int(map_height), int(robot_current_position.x - w / 2): int(map_width)], robot_surrounding_state[0:int(h / 2 + map_height - robot_current_position.y), 0:int(w / 2 + map_width - robot_current_position.x)])
        else:
            print("conflict~", temp_global_map[int(robot_current_position.y - h / 2):int(robot_current_position.y + h / 2), int(robot_current_position.x - int(w / 2)): int(robot_current_position.x + int(w / 2))].shape)
            print(robot_surrounding_state.shape  )
            temp_global_map[int(robot_current_position.y - h / 2):int(robot_current_position.y + h / 2), int(robot_current_position.x - int(w / 2)): int(robot_current_position.x + int(w / 2))] = cv2.bitwise_and(temp_global_map[int(robot_current_position.y - h / 2):int(robot_current_position.y + h / 2), int(robot_current_position.x - int(w / 2)): int(robot_current_position.x + int(w / 2))], robot_surrounding_state)
            # if robot_current_position.x < 0:
            #     temp_global_map[int(robot_current_position.y - h / 2):int(map_height), 0: int(w / 2 - robot_current_position.x)] = cv2.bitwise_and(temp_global_map[int(robot_current_position.y - h / 2):int(map_height), int(robot_current_position.x - w / 2): int(robot_current_position.x + w / 2)], robot_surrounding_state[0:int(h / 2 + map_height - robot_current_position.y), 0:w])
            # elif robot_current_position.x + w / 2 > map_width:
            #     temp_global_map[int(robot_current_position.y - h / 2):int(map_height), int(robot_current_position.x - w / 2): int(map_width)] = cv2.bitwise_and(temp_global_map[int(robot_current_position.y - h / 2):int(map_height), int(robot_current_position.x - w / 2): int(map_width)], robot_surrounding_state[0:int(h / 2 + map_height - robot_current_position.y), 0:int(w / 2 + map_width - robot_current_position.x)])
        self.convert_map_to_grid(temp_global_map)
        return temp_global_map

refPt = []
cropping = False

def click_and_crop(event, x, y, flags, param):
    global refPt, cropping

    if event == cv2.EVENT_LBUTTONDOWN:

        cropping = True
    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        # record the ending (x, y) coordinates and indicate that
        # the cropping operation is finished
        refPt.append((x, y))
        cropping = False

#
# cv2.namedWindow("map")
# cv2.setMouseCallback("map", click_and_crop)

if __name__ == "__main__":
    start = time.time()
    astar = Astar("../map_2.pgm")
    # astar.set_goal((415, 425))
    # astar.set_start((170, 651))
    # path = astar.define_path()
    # print(path)
    # for point in path:
    #     cv2.circle(astar.display_map, (point[1], point[0]), radius=0, color=(0, 0, 255), thickness=-1)
    # cv2.imshow('map', astar.display_map)
    # print(time.time() - start)

    while (1):
        if len(refPt) > 1:
            print(refPt, math.sqrt((refPt[0][0] - refPt[1][0]) ** 2 + (refPt[0][1] - refPt[1][1]) ** 2))
            start = time.time()
            # The order is Y, X
            astar.set_goal((refPt[1][1], refPt[1][0]))
            astar.set_start((refPt[0][1], refPt[0][0]))
            path = astar.define_path()
            for point in path:
                cv2.circle(astar.display_map, (point[1], point[0]), radius=0, color=(0, 0, 255), thickness=-1)
            print(time.time() - start)
            # cv2.rectangle(my_img, (30, 30), (300, 200), (0, 20, 200), 10)
            refPt = []
        cv2.imshow('map', astar.display_map)
        if cv2.waitKey(20) == 32:
            break
    cv2.waitKey(0)
