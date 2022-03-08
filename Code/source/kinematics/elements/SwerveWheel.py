import time
from typing import Union
from Wheel import Wheel


class SwerveWheel(Wheel):
    def __init__(self, radius: float, thickness: float, startingAngle: float = 0.0, swerveAngularVelocity: float = 0.0):
        super().__init__(radius, thickness)
        self.swerveAngle = startingAngle
        self.swerveAngularVelocity = swerveAngularVelocity
        self.swerveLastUpdated = time.perf_counter()

    def update(self, swerveAngularVelocity: Union[float, None] = None, wheelAngularVelocity: Union[float, None] = None):
        super().update(wheelAngularVelocity)
        self.swerveAngle += (time.perf_counter() - self.lastUpdated) * (self.angularVelocity * 6.0)
        if swerveAngularVelocity is not None:
            self.swerveAngularVelocity = swerveAngularVelocity
        self.swerveLastUpdated = time.perf_counter()
