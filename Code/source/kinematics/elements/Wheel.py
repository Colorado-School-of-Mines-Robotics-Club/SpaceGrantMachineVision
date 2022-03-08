from typing import Union, List, Tuple
from math import pi
import time


class Wheel:
    def __init__(self, radius: float, thickness: float, angularVelocity: float = 0.0, angle: float = 0.0):
        self.angularVelocity = angularVelocity
        self.angle = angle
        self.radius = radius
        self.thickness = thickness
        self.circumference = 2.0 * pi * self.radius
        self.lastUpdated = time.perf_counter()

    def update(self, angularVelocity: Union[float, None] = None):
        self.angle += (time.perf_counter() - self.lastUpdated) * (self.angularVelocity * 6.0)  # rpm to deg/sec is * 6.0
        if angularVelocity is not None:
            self.angularVelocity = angularVelocity
        self.lastUpdated = time.perf_counter()

    def getDistance(self):
        return self.circumference * (self.angle / 360.0)
