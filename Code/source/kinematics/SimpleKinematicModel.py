from typing import Tuple, List, Union
import math

try:
    from .WheelAssembly import WheelAssembly
    from .elements.RigidChassis import RigidChassis
    from source.utilities import Config
except ModuleNotFoundError:
    from .WheelAssembly import WheelAssembly
    from .elements.RigidChassis import RigidChassis
    from Code.source.utilities import Config

class SimpleKinematicModel:
    def __init__(self) -> None:
        Config.init()
        dimensions = Config.getDimensionsDict()
        wheelDim = dimensions['wheel']
        self.minSwerveAngle = wheelDim['minTheta']
        self.maxSwerveAngle = wheelDim['maxTheta']
        suspensionDim = dimensions['suspension']
        self.minSuspensionAngle = suspensionDim['minTheta']
        self.maxSuspensionAngle = suspensionDim['maxTheta']

        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        self.swerve_angles = [0.0, 0.0, 0.0, 0.0]
        self.suspension_heights = [0.0, 0.0, 0.0, 0.0]
    
    def update(self, wheelVelocities: List[Union[float, None]] = None, swerveAngles: List[Union[float, None]] = None,
        suspensionHeightTargets: List[Union[float, None]] = None) -> None:
        for i in range(4):
            self.wheel_velocities[i] = wheelVelocities[i]
        for i in range(4):
            self.swerve_angles[i] = swerveAngles[i]
            # self.swerve_angles[i] = self.minSwerveAngle if swerveAngles[i] < self.minSwerveAngle else self.maxSwerveAngle if swerveAngles[i] > self.maxSwerveAngle else swerveAngles[i]
        for i in range(4):
            self.suspension_heights[i] = suspensionHeightTargets[i]
            # self.suspension_heights[i] = 10 #self.minSuspensionAngle if suspensionHeightTargets[i] < self.minSuspensionAngle else self.maxSuspensionAngle if suspensionHeightTargets[i] > self.maxSuspensionAngle else suspensionHeightTargets[i]
    
    def getWheelVelocities(self) -> List[float]:
        return self.wheel_velocities

    def getSwerveWheelAngles(self) -> List[float]:
        return self.swerve_angles

    def getSuspensionAngles(self) -> List[float]:
        return self.suspension_heights