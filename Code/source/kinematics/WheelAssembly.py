from .Wheel import Wheel
from .Suspension import Suspension


class WheelAssembly:
    def __init__(self):
        self.wheel = Wheel()
        self.suspension = Suspension()
        return

    def update(self):
        return
