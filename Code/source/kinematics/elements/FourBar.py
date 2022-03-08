from typing import Union
import math
from KinematicObject import KinematicObject
import numpy as np


class FourBar(KinematicObject):
    def __init__(self, length: float, height: float, angle: float = 0.0, maxAngle: Union[float, None] = None,
                 minAngle: Union[float, None] = None):
        super().__init__(angles=(0.0, angle, 0.0))
        self.length = length
        self.height = height
        self.offsetHeight = 0.0
        self.offsetWidth = self.length
        self.maxAngle = maxAngle
        self.minAngle = minAngle
        self.update()

    def update(self, angle: Union[float, None] = None) -> None:
        if angle is not None:
            super().setPose(angles=(0.0, self.constrainAngleInput(angle), 0.0))
        self.offsetHeight = math.sin(super().angles[1]) * self.length
        self.offsetWidth = math.sin(90.0 - super().angles[1]) * self.length

    def constrainAngleInput(self, angle: float) -> float:
        if self.minAngle is not None and self.maxAngle is not None:
            return np.clip(angle, self.minAngle, self.maxAngle)
        return angle
