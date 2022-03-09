from typing import Union
from .elements.ParallelogramFourBar import ParallelogramFourBar


class Suspension(ParallelogramFourBar):
    def __init__(self, length: float, height: float, angle: float = 0.0, maxAngle: Union[float, None] = None,
                 minAngle: Union[float, None] = None):
        super().__init__(length=length, height=height, angle=angle, maxAngle=maxAngle, minAngle=minAngle)

    def update(self, angle: Union[float, None], height: Union[float, None] = None, width: Union[float, None] = None,
               inverse: bool = True):
        if inverse:
            if height is None and width is None:
                print("WARNING -> Suspension -> update: Inverse is True, but height and width targets not given")
            super().inverseUpdate(offsetHeight=height, offsetWidth=width)
        else:
            if angle is None:
                print("WARNING -> Suspension -> update: Inverse is False, but angle not given")
            super().forwardUpdate(angle=angle)
