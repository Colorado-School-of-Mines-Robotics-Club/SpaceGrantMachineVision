from typing import Union, Tuple
from math import pi
from KinematicObject import KinematicObject


class Wheel(KinematicObject):
    def __init__(self, radius: float, thickness: float, wheelAngularVelocity: Union[float, None] = None,
                 startingRotation: Union[float, None] = None,
                 position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angles: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularVelocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularAcceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
        angleY = startingRotation if (startingRotation is not None) else angles[1]
        angularVelocityY = wheelAngularVelocity if (wheelAngularVelocity is not None) else angularVelocity[1]
        super().__init__(position=position, angles=(angles[0], angleY, angles[2]), velocity=velocity,
                         angularVelocity=(angularVelocity[0], angularVelocityY, angularVelocity[2]),
                         acceleration=acceleration, angularAcceleration=angularAcceleration)
        self.radius = radius
        self.thickness = thickness
        self.circumference = 2.0 * pi * self.radius

    def update(self, angularVelocity: Union[float, None] = None, angularAcceleration: Union[float, None] = None):
        angularVelocity = (0.0, angularVelocity, 0.0) if (angularVelocity is not None) else None
        angularAcceleration = (0.0, angularAcceleration, 0.0) if (angularAcceleration is not None) else None
        super().updateRotation(angularVelocity=angularVelocity, angularAcceleration=angularAcceleration)

    def getDistance(self):
        return self.circumference * (super().angles[1] / 360.0)
