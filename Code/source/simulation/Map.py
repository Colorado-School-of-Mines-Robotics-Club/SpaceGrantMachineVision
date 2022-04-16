# Built in python libs
import sys
import os
from typing import Dict, List, Tuple

# Additional libs
import cv2
import numpy as np
import math
import random

# Custom imports
from bezier import Curve

class Map:
    def __init__(self, nodeLayout=(100, 51), scaleFactor=9, D=1):
        self.nodeLayout = nodeLayout
        self.scaleFactor = scaleFactor
        self.imageSize = (self.nodeLayout[0] * self.scaleFactor, self.nodeLayout[1] * self.scaleFactor, 3)
        self.grid = np.zeros((self.nodeLayout[0], self.nodeLayout[1], 3))
        self.DPerNode = float(D) / (self.nodeLayout[1])

    def instruction_converter(self, route, compress=False):
        operatives = []
        direction = 0

        for i in range(len(route) - 1):
            start = route[i]
            end = route[i + 1]

            if end[0] - start[0] > 0:
                new_dir = 180
            elif end[0] - start[0] < 0:
                new_dir = 0
            elif end[1] - start[1] > 0:
                new_dir = 90
            elif end[1] - start[1] < 0:
                new_dir = -90
            else:
                new_dir = direction

            if direction != new_dir:
                operatives.append(("ANG", new_dir - direction))
                direction = new_dir

            operatives.append(["LIN", self.DPerNode])

        if compress:
            compressed = []

            for i in operatives:
                if len(compressed) == 0:
                    compressed.append(i)
                    continue

                if i[0] == "LIN" and compressed[-1][0] == "LIN":
                    compressed[-1][1] += i[1]
                elif i[0] == "LIN" and compressed[-1][0] == "ANG":
                    compressed.append(i)
                elif i[0] == "ANG" and compressed[-1][0] == "LIN":
                    compressed.append(i)

            return compressed
        else:
            return operatives
    
    #UNFINISHED
    def bezier_to_PID(route, curr_pos, curr_ang, Kp = 1, Kd = 1, Ki = 1, Kh = 1, Khe = 1, v_max = 2, ang_max = 0.25, ts = 50):
        csum = 0
        for i in range(ts):
            dist = 1000000000000
            point = 0
            for i in range(len(route)):
                euc = (curr_pos[1] - route[i][1])**2 + (curr_pos[0] - route[i][0]) ** 2
                if euc < dist:
                    dist = euc
                    point = i
            
            segment = [route[point], route[point+1]]

            vs = np.array([segment[1][0] - segment[0][0], segment[1][1] - segment[0][1]])
            vr = np.array([curr_pos[0] - segment[0][0], curr_pos[1] - segment[0][1]])

            theta_p = math.atan2((segment[1][1] - segment[0][1])/(segment[1][0] - segment[0][0]))
            theta_pr = theta_p + math.pi

            R = np.array([[math.cos(theta_pr), -math.sin(theta_pr)], [math.sin(theta_pr), math.cos(theta_pr)]])

            vr_rot = np.matmul(R,vr)

            cte = vr_rot[1]
            cte = max(cte, -Kh)
            cte = min(cte, Kh)

            theta_setpoint = theta_p + ((cte/Kh) * (math.pi/2))
            theta_err = theta_setpoint - curr_ang
            v_setpoint = max(v_max - Khe * theta_err, 0)

            csum += theta_err
            ang_setpoint = Kp * (theta_err) + Ki * csum * ts + Kd * ts


        return None

    # takes an optional color
    #       0 - blue
    #       1 - green
    #       2 - red
    def updatePoint(self, x: int, y: int, score=1, color=2):
        self.grid[x][y][color] += score

    def convert_route_to_dist(self, route):
        dist = [[i[0] * self.DPerNode, i[1] * self.DPerNode] for i in route]
        displacement = []
        center = dist[0]
        for i in dist:
            displacement.append([center[0] - i[0], i[1] - center[1]])
        return displacement

    def big_bezzy(self, route):
        #Input: route to shorten
        #Output: The mother of all Bezier Curves

        dist = ((route[-1][1] - route[0][1])**2 + (route[-1][0] - route[0][0])**2)**0.5
        NUM_OF_POINTS = round(dist) * 2

        curve = Curve(route.T, degree = len(route) - 1)
        points = [X/NUM_OF_POINTS for X in range(0,NUM_OF_POINTS + 1)]
        smoothened = [[curve.evaluate(i)[0][0],curve.evaluate(i)[1][0]] for i in points]

        return smoothened

    def draw(self) -> np.ndarray:
        display = np.zeros(self.imageSize, dtype='uint8')
        offset = math.ceil(self.scaleFactor / 2)
        for row in range(self.nodeLayout[0]):
            y = row * self.scaleFactor + offset
            # cv2.line(display, (0, y), (self.imageSize[0], y), (255, 255, 255))
            for col in range(self.nodeLayout[1]):
                color = (self.grid[row][col][0], self.grid[row][col][1], self.grid[row][col][2])
                x = col * self.scaleFactor + offset
                cv2.circle(display, (x, y), radius=math.ceil(offset / 2), color=color, thickness=-1)
        return display

    def assign_rand(self, max_value=30):
        x = random.randrange(0, self.nodeLayout[0])
        y = random.randrange(0, self.nodeLayout[1])
        color = random.randrange(0, 3)
        value = random.randrange(0, max_value)

        self.updatePoint(x, y, score=value, color=color)

    def get_grid(self) -> np.ndarray:
        b, g, r = cv2.split(self.grid)
        return r

    def get_passable(self) -> np.ndarray:
        b, g, r = cv2.split(self.grid)
        return g

    def poseToNode(self, x: float, z: float) -> Tuple[int, int]:
        x = int(x * self.DPerNode)
        z = int(z * self.DPerNode)

        # want to offset position to a node value
        x += math.floor(self.nodeLayout[0] / 2) + 1
        z = self.nodeLayout[1] - z

        return x, z  # final tuple format for current node

    def getEndNode(self) -> Tuple[int, int]:
        return math.floor(self.nodeLayout[0] / 2) + 1, int(self.nodeLayout[1] / 2)
