from typing import Union, Tuple
from .Wheel import Wheel
import math


class SwerveWheel(Wheel):
    def __init__(self, radius: float, thickness: float = 1.0,
                 wheelRotation: Union[float, None] = None,
                 wheelAngularVelocity: Union[float, None] = None,
                 wheelAcceleration: Union[float, None] = None,
                 swerveAngle: Union[float, None] = None, swerveOffset: Union[float, None] = None,
                 swerveMaxAngle: Union[float, None] = None, swerveMinAngle: Union[float, None] = None,
                 position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angles: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularVelocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularAcceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self.maxA = swerveMaxAngle
        self.minA = swerveMinAngle
        self.posOffset = swerveOffset
        swerveAngle = 0.0 if swerveAngle is None else swerveAngle
        rotationZ = super().constrain(swerveAngle, self.minA, self.maxA) if (swerveAngle is not None) else 0.0
        x = math.sin(swerveAngle) * self.posOffset
        y = math.cos(swerveAngle) * self.posOffset
        super().__init__(radius=radius, thickness=thickness, angles=(angles[0], angles[1], rotationZ),
                         position=(x, y, position[2]), velocity=velocity, acceleration=acceleration,
                         wheelRotation=wheelRotation, wheelAngularVelocity=wheelAngularVelocity,
                         wheelAngularAcceleration=wheelAcceleration,
                         angularVelocity=angularVelocity, angularAcceleration=angularAcceleration)

    def getSwerveAngle(self) -> float:
        return super().angles[2]

    def update(self, swerveAngle: Union[float, None] = None, wheelAngularVelocity: Union[float, None] = None,
               wheelAngularAcceleration: Union[float, None] = None,
               velocity: Union[Tuple[float, float, float], None] = None,
               acceleration: Union[Tuple[float, float, float], None] = None,
               angularVelocity: Union[Tuple[float, float, float], None] = None,
               angularAcceleration: Union[Tuple[float, float, float], None] = None) -> None:
        super().update(wheelAngularVelocity, wheelAngularAcceleration, velocity, acceleration, angularVelocity,
                       angularAcceleration)
        if swerveAngle is not None:
            swerveAngle = super().constrain(swerveAngle, self.minA, self.maxA)
            x = math.sin(swerveAngle) * self.posOffset
            y = math.cos(swerveAngle) * self.posOffset
            super().setPose(angles=(0.0, self.angles[1], swerveAngle), position=(x, y, 0.0))
