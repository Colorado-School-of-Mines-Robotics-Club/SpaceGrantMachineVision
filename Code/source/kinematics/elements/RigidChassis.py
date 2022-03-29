from typing import Tuple, Dict, Any, List
from .KinematicObject import KinematicObject


class RigidChassis(KinematicObject):
    def __init__(self, dimensions: Tuple[float, float, float],
                 mountingPoints: List[Tuple[float, float, float]] = None,
                 position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angles: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularVelocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 angularAcceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
        super().__init__(position, angles, velocity, angularVelocity, acceleration, angularAcceleration)
        self.dimensions = dimensions
        self.mountingPoints = mountingPoints

    def update(self, velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
               angularVelocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
               acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0),
               angularAcceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        super().update(velocity, angularVelocity, acceleration, angularAcceleration)
        # need to update the point in space based on the chassis position and rotation
        for point in self.mountingPoints:
            pass
