from typing import Tuple, Union
import math


class FourBar:
    def __init__(self, length: float, height: float, angle: float = 0.0):
        self.length = length
        self.height = height
        self.angle = angle

        self.offsetHeight = 0.0
        self.offsetWidth = self.length

        self.update()

    def update(self, angle: Union[float, None] = None):
        if angle is not None:
            self.angle = angle
        self.offsetHeight = math.sin(self.angle) * self.length
        self.offsetWidth = math.sin(90.0 - self.angle) * self.length
