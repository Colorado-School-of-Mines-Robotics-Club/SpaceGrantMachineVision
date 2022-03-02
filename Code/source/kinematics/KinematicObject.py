from typing import Tuple


class KinematicObject:
    def __init__(self, velocity: Tuple[float, float, float], acceleration: Tuple[float, float, float],
                 position: Tuple[float, float, float]):
        self.vel = velocity
        self.acc = acceleration
        self.pos = position

    def update(self):
        return
    