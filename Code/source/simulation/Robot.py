# Built in python libs
import sys
import os
import time
from typing import Dict, List
from queue import Queue
from math import ceil, cos, sin

# Additional libs
import cv2
import numpy as np

# Custom imports


class Robot:
    # worldScale defines the m / node
    def __init__(self, startingNode=(99, 25), width=16, height=24, scaleFactor=9, worldScale=1.0):
        self.gridX = startingNode[1]
        self.gridY = startingNode[0]
        self.scaleFactor = scaleFactor
        self.width = width
        self.height = height
        self.velocity = 0
        self.velocityUpdateTime = 0
        self.prevVelocities = Queue()  # stores previous velocities and their time interval active
        self.angularVelocity = 0
        self.angularVelocityUpdateTime = 0
        self.prevAngularVelocities = Queue()  # stores previous angular velocities and their time interval active
        self.theta = 0  # measured relative to 0 deg being straight vertical in the simulation map

    def updateVelocity(self, velocity: float):
        if self.velocityUpdateTime != 0:
            self.prevVelocities.put((self.velocity, time.perf_counter() - self.velocityUpdateTime))
        self.velocity = velocity
        self.velocityUpdateTime = time.perf_counter()

    def updateAngularVelocity(self, angularVelocity: float):
        if self.angularVelocityUpdateTime != 0:
            self.prevAngularVelocities.put((self.angularVelocity, time.perf_counter() - self.angularVelocityUpdateTime))
        self.angularVelocity = angularVelocity
        self.angularVelocityUpdateTime = time.perf_counter()

    def draw(self, display: np.ndarray) -> np.ndarray:
        offset = ceil(self.scaleFactor / 2)
        centerX, centerY = self.gridX * self.scaleFactor + offset, self.gridY * self.scaleFactor + offset
        x1, y1 = centerX - self.width / 2, centerY - self.height / 2
        x2, y2 = centerX + self.width / 2, centerY + self.height / 2
        rx1 = centerX + (x1 * cos(self.theta)) - (y1 * sin(self.theta))
        ry1 = centerY + (x1 * sin(self.theta)) + (y1 * cos(self.theta))
        rx2 = centerX + (x2 * cos(self.theta)) - (y2 * sin(self.theta))
        ry2 = centerY + (x2 * sin(self.theta)) + (y2 * cos(self.theta))
        p1, p2 = (rx1, ry1), (rx2, ry2)
        cv2.rectangle(display, p1, p2, (170, 170, 170), -1)
        return display

