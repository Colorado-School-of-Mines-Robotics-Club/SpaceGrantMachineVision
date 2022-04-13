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

    # takes an optional color
    #       0 - blue
    #       1 - green
    #       2 - red
    def updatePoint(self, x: int, y: int, score=1, color=2):
        self.grid[x][y][color] += score

    def convert_route_to_dist(self, route):
        return [list(i * self.DPerNode) for i in route]

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

    def incrementNodeScoresFromPose(self, objects: List) -> None:
        for wx, wy, wz in objects:
            x, y = self.poseToNode(wx, wz)
            try:
                self.updatePoint(x, y, score=5)
            except IndexError:
                pass
