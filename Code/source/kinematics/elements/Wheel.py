from typing import Union
from math import pi
from KinematicObject import KinematicObject


class Wheel(KinematicObject):
    def __init__(self, radius: float, thickness: float, angularVelocity: float = 0.0, startingRotation: float = 0.0):
        super().__init__(angularVelocity=(0.0, angularVelocity, 0.0), angles=(0.0, startingRotation, 0.0))
        self.radius = radius
        self.thickness = thickness
        self.circumference = 2.0 * pi * self.radius

    def update(self, angularVelocity: Union[float, None] = None):
        if angularVelocity is None:
            super().updateRotation()
        else:
            super().updateRotation(angularVelocity=(0.0, angularVelocity, 0.0))

    def getDistance(self):
        return self.circumference * (super().angles[1] / 360.0)
