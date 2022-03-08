from .elements.Wheel import Wheel
from Suspension import Suspension


class WheelAssembly:
    def __init__(self, wheelRadius: float, wheelThickness: float, suspensionLength: float, suspensionHeight: float):
        self.wheel = Wheel(wheelRadius, wheelThickness)
        self.suspension = Suspension(suspensionLength, suspensionHeight)

    def update(self):
        return
