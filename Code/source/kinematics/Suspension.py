from typing import Union
from .elements.ParallelogramFourBar import ParallelogramFourBar


class Suspension(ParallelogramFourBar):
    def __init__(self, length: float, height: float, angle: float = 0.0, maxAngle: Union[float, None] = None,
                 minAngle: Union[float, None] = None):
        super().__init__(length=length, height=height, angle=angle, maxAngle=maxAngle, minAngle=minAngle)

    def update(self, angle: Union[float, None], height: Union[float, None] = None, width: Union[float, None] = None,
               inverse: bool = True):
        super().update(angle=angle, offsetHeight=height, offsetWidth=width)
