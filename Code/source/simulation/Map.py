# Built in python libs
import sys
import os
from typing import Dict, List

# Additional libs
import cv2
import numpy as np
import math
import random

# Custom imports
from source.logger.Logger import Logger
from source.utilities.Config import Config


class Map:
    def __init__(self, nodeLayout=(100, 51), scaleFactor=9):
        self.nodeLayout = nodeLayout
        self.scaleFactor = scaleFactor
        self.imageSize = (self.nodeLayout[0]*self.scaleFactor, self.nodeLayout[1]*self.scaleFactor, 3)
        self.grid = np.zeros((self.nodeLayout[0], self.nodeLayout[1], 3))

    # takes an optional color
    #       0 - blue
    #       1 - green
    #       2 - red
    def updatePoint(self, x: int, y: int, score=1, color=2):
        self.grid[x][y][color] += score

    def draw(self) -> np.ndarray:
        display = np.zeros(self.imageSize, dtype='uint8')
        offset = math.ceil(self.scaleFactor / 2)
        for row in range(self.nodeLayout[0]):
            y = row*self.scaleFactor + offset
            # cv2.line(display, (0, y), (self.imageSize[0], y), (255, 255, 255))
            for col in range(self.nodeLayout[1]):
                color = (self.grid[row][col][0], self.grid[row][col][1], self.grid[row][col][2])
                x = col*self.scaleFactor + offset
                cv2.circle(display, (x, y), radius=math.ceil(offset / 2), color=color, thickness=-1)
        return display
    
    def assign_rand(self):
        danger = random.randint(0,2)
        x = random.randrange(0, self.nodeLayout[0])
        y = random.randrange(0, self.nodeLayout[1])

        for i in range(3):
            self.grid[x][y][i] = 0
        
        self.grid[x][y][danger] = 1  
    
    def convert(self) -> np.ndarray:
        flattened = np.zeros((self.nodeLayout[0], self.nodeLayout[1]))
        for x in self.grid:
            for y in x:
                for color in range(3):
                    if self.grid[x][y][color] == 1:
                        flattened[x][y] = color
        return flattened