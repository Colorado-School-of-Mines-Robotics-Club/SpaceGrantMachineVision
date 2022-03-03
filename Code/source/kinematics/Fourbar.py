from typing import Tuple, Union
from source.utilities.Config import Config


class Fourbar:
    def __init__(self, length: float, height: float, angle: float):
        self.length = length
        self.height = height
        self.angle = angle

        self.update()

    def update(self, angle: Union[float, None] = None):
        if angle is not None:
            self.angle = angle

