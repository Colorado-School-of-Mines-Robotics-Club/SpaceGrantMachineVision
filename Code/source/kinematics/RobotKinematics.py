from .WheelAssembly import WheelAssembly
from .KinematicObject import KinematicObject
from typing import Tuple


class RobotKinematics(KinematicObject):
    def __init__(self, velocity: Tuple[float, float, float], acceleration: Tuple[float, float, float],
                 position: Tuple[float, float, float]):
        super().__init__(velocity, acceleration, position)
        self.wheels = (WheelAssembly(), WheelAssembly(), WheelAssembly(), WheelAssembly())
        return

    def update(self):
        return
