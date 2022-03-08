from typing import Union, Tuple
from KinematicObject import KinematicObject
import math


class ParallelogramFourBar(KinematicObject):
    def __init__(self, length: float, height: float, angle: float = 0.0, maxAngle: Union[float, None] = None,
                 minAngle: Union[float, None] = None):
        self.maxAngle = maxAngle
        self.minAngle = minAngle
        angle = super().constrain(angle, self.minAngle, self.maxAngle) if (angle is not None) else 0.0
        super().__init__(angles=(0.0, angle, 0.0))
        self.length = length
        self.height = height
        self.offsetHeight = 0.0
        self.offsetWidth = self.length
        self.update()

    def update(self, angle: Union[float, None] = None) -> None:
        if angle is not None:
            super().setPose(angles=(0.0, super().constrain(angle, self.minAngle, self.maxAngle), 0.0))
        self.offsetHeight = math.sin(super().angles[1]) * self.length
        self.offsetWidth = math.sin(90.0 - super().angles[1]) * self.length

    def getPos(self) -> Tuple[float, float]:
        return self.offsetWidth, self.offsetHeight
