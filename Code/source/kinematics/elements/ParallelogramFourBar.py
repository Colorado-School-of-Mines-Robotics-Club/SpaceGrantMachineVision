from typing import Union, Tuple
from .KinematicObject import KinematicObject
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
        self.forwardUpdate()

    def forwardUpdate(self, angle: Union[float, None] = None) -> None:
        if angle is not None:
            super().setPose(angles=(0.0, super().constrain(angle, self.minAngle, self.maxAngle), 0.0))
        self.offsetHeight = math.sin(self.angles[1]) * self.length
        self.offsetWidth = math.sin((math.pi / 2.0) - self.angles[1]) * self.length

    def inverseUpdate(self, offsetHeight: Union[float, None] = None, offsetWidth: Union[float, None] = None) -> None:
        targetAngle = None
        if offsetHeight is not None:
            inputAngle = super().constrain(offsetHeight / self.length, -1.0, 1.0)
            targetAngle = math.asin(inputAngle)
        elif offsetWidth is not None:
            inputAngle = super().constrain(offsetWidth / self.length, -1.0, 1.0)
            targetAngle = -1.0 * (math.asin(inputAngle) - (math.pi / 2.0))
        self.forwardUpdate(targetAngle)

    def update(self, angle: Union[float, None] = None, offsetHeight: Union[float, None] = None,
               offsetWidth: Union[float, None] = None, inverse: bool = True) -> None:
        super().update()
        if inverse:
            self.inverseUpdate(offsetHeight=offsetHeight, offsetWidth=offsetWidth)
        else:
            self.forwardUpdate(angle=angle)

    def getAngle(self, radians=False) -> float:
        if radians:
            return self.angles[1]
        return self.angles[1] * (180.0 / math.pi)

    def getPos(self) -> Tuple[float, float]:
        return self.offsetWidth, self.offsetHeight

    @staticmethod
    def staticInverseUpdate(length: float, targetHeight: float, maxAngle: float, minAngle: float):
        targetAngle = super().constrain(math.asin(targetHeight / length), minAngle, maxAngle)
        return targetAngle
