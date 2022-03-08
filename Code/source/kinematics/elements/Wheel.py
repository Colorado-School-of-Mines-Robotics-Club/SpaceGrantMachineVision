from typing import Union, List, Tuple
from math import pi
import time
from KinematicObject import KinematicObject


class Wheel(KinematicObject):
    def __init__(self, radius: float, thickness: float, angularVelocity: float = 0.0, startingRotation: float = 0.0):
        super().__init__(angularVelocity=(0.0, angularVelocity, 0.0), angles=(0.0, startingRotation, 0.0))
        self.angularVelocity = angularVelocity
        self.totalRotation = startingRotation
        self.radius = radius
        self.thickness = thickness
        self.circumference = 2.0 * pi * self.radius
        self.lastUpdated = time.perf_counter()

    def update(self, angularVelocity: Union[float, None] = None):
        self.totalRotation += (time.perf_counter() - self.lastUpdated) * (self.angularVelocity * 6.0)
        if angularVelocity is not None:
            self.angularVelocity = angularVelocity
        self.lastUpdated = time.perf_counter()

    def getDistance(self):
        return self.circumference * (self.totalRotation / 360.0)
