import time
from typing import Tuple, Union


class KinematicObject:
    """ Stores 3D position, velocity, and acceleration data for translation and rotation
        Standard units for translation are m, m/s, m/s^2
        Standard units for rotation are deg, rpm, rpm/s
    """
    def __init__(self, position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angles: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularVelocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularAcceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
        self.pos = position
        self.angles = angles
        self.vel = velocity
        self.angVel = angularVelocity
        self.acc = acceleration
        self.angAcc = angularAcceleration
        self.lastUpdatedPos = time.perf_counter()
        self.lastUpdatedAngles = time.perf_counter()

    def setPose(self, position: Tuple[float, float, float], angles: Tuple[float, float, float]):
        self.pos = position
        self.angles = angles

    def updateRotation(self, angularVelocity: Union[Tuple[float, float, float], None] = None,
                       angularAcceleration: Union[Tuple[float, float, float], None] = None):
        deltaT = time.perf_counter() - self.lastUpdatedAngles
        self.angles = (self.angles[0] + 6.0 * deltaT * self.angVel[0] + 6.0 * deltaT ** 2 * self.angAcc[0],
                       self.angles[1] + 6.0 * deltaT * self.angVel[1] + 6.0 * deltaT ** 2 * self.angAcc[1],
                       self.angles[2] + 6.0 * deltaT * self.angVel[2] + 6.0 * deltaT ** 2 * self.angAcc[2])
        if angularVelocity is not None:
            self.angVel = angularVelocity
        if angularAcceleration is not None:
            self.angAcc = angularAcceleration
        self.lastUpdatedAngles = time.perf_counter()

    def updatePosition(self, velocity: Union[Tuple[float, float, float], None] = None,
                       acceleration: Union[Tuple[float, float, float], None] = None):
        deltaT = time.perf_counter() - self.lastUpdatedPos
        self.pos = (self.pos[0] + deltaT * self.vel[0] + deltaT ** 2 * self.acc[0],
                    self.pos[1] + deltaT * self.vel[1] + deltaT ** 2 * self.acc[1],
                    self.pos[2] + deltaT * self.vel[2] + deltaT ** 2 * self.acc[2])
        if velocity is not None:
            self.vel = velocity
        if acceleration is not None:
            self.acc = acceleration
        self.lastUpdatedPos = time.perf_counter()
