from typing import Union, Tuple
from Wheel import Wheel


# TODO, actual robot has an offset from the Z axis for its 'swerve' rotation, will need to account for this in the
# call to super().setPose with a calculated x,y position (super().setPose in SwerveWheel.update)
class SwerveWheel(Wheel):
    def __init__(self, radius: float, thickness: float = 1.0,
                 wheelRotation: Union[float, None] = None,
                 wheelAngularVelocity: Union[float, None] = None,
                 wheelAcceleration: Union[float, None] = None,
                 swerveAngle: Union[float, None] = None,
                 swerveMaxAngle: Union[float, None] = None, swerveMinAngle: Union[float, None] = None,
                 position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angles: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularVelocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularAcceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self.maxA = swerveMaxAngle
        self.minA = swerveMinAngle
        rotationZ = super().constrain(swerveAngle, self.minA, self.maxA) if (swerveAngle is not None) else 0.0
        super().__init__(radius=radius, thickness=thickness, angles=(angles[0], angles[1], rotationZ),
                         position=position, velocity=velocity, acceleration=acceleration,
                         wheelRotation=wheelRotation, wheelAngularVelocity=wheelAngularVelocity,
                         wheelAngularAcceleration=wheelAcceleration,
                         angularVelocity=angularVelocity, angularAcceleration=angularAcceleration)

    def update(self, swerveAngle: Union[float, None] = None, wheelAngularVelocity: Union[float, None] = None,
               wheelAngularAcceleration: Union[float, None] = None) -> None:
        super().update(angularVelocity=wheelAngularVelocity, angularAcceleration=wheelAngularAcceleration)
        if swerveAngle is not None:
            super().setPose(angles=(0.0, super().angles[1], super().constrain(swerveAngle, self.minA, self.maxA)))
