from .WheelAssembly import WheelAssembly
from .Kinematic import Kinematic


class RobotKinematics(Kinematic):
    def __init__(self):
        self.wheels = (WheelAssembly(), WheelAssembly(), WheelAssembly(), WheelAssembly())
        return

    def update(self):
        return
