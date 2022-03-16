from .WheelAssembly import WheelAssembly
from elements.KinematicObject import KinematicObject
from elements.RigidChassis import RigidChassis
from typing import Tuple, List, Union

from source.utilities import Config


class BottermelonKinematics:
    def __init__(self):
        # read data in from the config file
        Config.init()
        dimensions = Config.getDimensionsDict()

        # setup the chassis
        chasDim = dimensions['chassis']
        matchingPoints = None  # TODO define mounting points and define update for them. Not urgent
        self.chassis = RigidChassis(dimensions=(chasDim['width'] / 1000.0, chasDim['length'] / 1000.0,
                                                chasDim['height'] / 1000.0),
                                    mountingPoints=matchingPoints)

        # setup the wheels
        suspensionDim = dimensions['suspension']
        wheelDim = dimensions['wheel']
        front_left_wheel = WheelAssembly(susHeight=suspensionDim['height'] / 1000.0,
                                         susLength=suspensionDim['length'] / 1000.0,
                                         susMaxAngle=suspensionDim['maxTheta'],
                                         susMinAngle=suspensionDim['minTheta'],
                                         wheelRadius=wheelDim['radius'] / 1000.0,
                                         wheelThickness=wheelDim['thickness'],
                                         swerveOffset=wheelDim['offset'],
                                         swerveMaxAngle=wheelDim['minTheta'],
                                         swerveMinAngle=wheelDim['maxTheta'])
        front_right_wheel = WheelAssembly(susHeight=suspensionDim['height'] / 1000.0,
                                          susLength=suspensionDim['length'] / 1000.0,
                                          susMaxAngle=suspensionDim['maxTheta'],
                                          susMinAngle=suspensionDim['minTheta'],
                                          wheelRadius=wheelDim['radius'] / 1000.0,
                                          wheelThickness=wheelDim['thickness'],
                                          swerveOffset=wheelDim['offset'],
                                          swerveMaxAngle=wheelDim['maxTheta'],
                                          swerveMinAngle=wheelDim['minTheta'])
        back_left_wheel = WheelAssembly(susHeight=suspensionDim['height'] / 1000.0,
                                        susLength=suspensionDim['length'] / 1000.0,
                                        susMaxAngle=suspensionDim['maxTheta'],
                                        susMinAngle=suspensionDim['minTheta'],
                                        wheelRadius=wheelDim['radius'] / 1000.0,
                                        wheelThickness=wheelDim['thickness'],
                                        swerveOffset=wheelDim['offset'],
                                        swerveMaxAngle=wheelDim['maxTheta'],
                                        swerveMinAngle=wheelDim['minTheta'])
        back_right_wheel = WheelAssembly(susHeight=suspensionDim['height'] / 1000.0,
                                         susLength=suspensionDim['length'] / 1000.0,
                                         susMaxAngle=suspensionDim['maxTheta'],
                                         susMinAngle=suspensionDim['minTheta'],
                                         wheelRadius=wheelDim['radius'] / 1000.0,
                                         wheelThickness=wheelDim['thickness'],
                                         swerveOffset=wheelDim['offset'],
                                         swerveMaxAngle=wheelDim['minTheta'],
                                         swerveMinAngle=wheelDim['maxTheta'])
        self.wheels = (front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel)

    def update(self, wheelVelocities: List[Union[float, None]], swerveAngles: List[Union[float, None]],
               suspensionHeightTargets: List[Union[float, None]]):
        for idx, wheel in enumerate(self.wheels):
            wheel.update(swerveAngle=swerveAngles[idx], wheelVelocity=wheelVelocities[idx],
                         suspensionTarget=suspensionHeightTargets[idx])
        # TODO, not urgent
        # make a determination of the velocity from wheels and/or other math.
        self.chassis.update()
