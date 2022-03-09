from typing import Union
from .elements.SwerveWheel import SwerveWheel
from Suspension import Suspension


class WheelAssembly:
    def __init__(self, susLength: float, susHeight: float, susAngle: float = 0.0,
                 susMaxAngle: Union[float, None] = None, susMinAngle: Union[float, None] = None,
                 wheelRadius: Union[float, None] = None, wheelThickness: Union[float, None] = None,
                 swerveOffset: Union[float, None] = None):
        self.wheel = SwerveWheel(radius=wheelRadius, thickness=wheelThickness, swerveOffset=swerveOffset)
        self.suspension = Suspension(length=susLength, height=susHeight, angle=susAngle, maxAngle=susMaxAngle,
                                     minAngle=susMinAngle)

    def update(self):
        self.wheel.update()
        self.suspension.update()
