from typing import Union, Tuple
from math import pi
from KinematicObject import KinematicObject


class Wheel(KinematicObject):
    def __init__(self, radius: float, thickness: float = 1.0,
                 wheelRotation: Union[float, None] = None,
                 wheelAngularVelocity: Union[float, None] = None,
                 wheelAngularAcceleration: Union[float, None] = None,
                 position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angles: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularVelocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularAcceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
        angleY = wheelRotation if (wheelRotation is not None) else angles[1]
        angularVelocityY = wheelAngularVelocity if (wheelAngularVelocity is not None) else angularVelocity[1]
        angularAccelerationY = wheelAngularAcceleration if (wheelAngularAcceleration is not None) else\
            angularAcceleration[1]
        super().__init__(position=position, angles=(angles[0], angleY, angles[2]), velocity=velocity,
                         angularVelocity=(angularVelocity[0], angularVelocityY, angularVelocity[2]),
                         acceleration=acceleration,
                         angularAcceleration=(angularAcceleration[0], angularAccelerationY, angularAcceleration[2]))
        self.radius = radius
        self.thickness = thickness
        self.circumference = 2.0 * pi * self.radius

    def update(self, wheelAngularVelocity: Union[float, None] = None,
               wheelAngularAcceleration: Union[float, None] = None,
               velocity: Union[Tuple[float, float, float], None] = None,
               acceleration: Union[Tuple[float, float, float], None] = None,
               angularVelocity: Union[Tuple[float, float, float], None] = None,
               angularAcceleration: Union[Tuple[float, float, float], None] = None) -> None:
        angularVelocity = (0.0, angularVelocity, 0.0) if (wheelAngularVelocity is not None) else angularVelocity
        angularAcceleration = (0.0, angularAcceleration, 0.0) if (wheelAngularAcceleration is not None) else\
            angularAcceleration
        super().updateRotation(angularVelocity=angularVelocity, angularAcceleration=angularAcceleration)

    def overrideAngle(self, angle: float) -> None:
        super().angles = (super().angles[0], angle, super().angles[2])

    def getDistance(self) -> float:
        return self.circumference * (super().angles[1] / 360.0)
