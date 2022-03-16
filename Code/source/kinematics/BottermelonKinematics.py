from .WheelAssembly import WheelAssembly
from elements.KinematicObject import KinematicObject
from elements.RigidChassis import RigidChassis
from typing import Tuple

from source.utilities import Config


class BottermelonKinematics(KinematicObject):
    def __init__(self):
        # init the kinematicObject, since everything should be 0 to start leave constructor blank
        super().__init__()

        # read data in from the config file
        Config.init()
        dimensions = Config.getDimensionsDict()

        # setup the chassis
        chasDim = dimensions['chassis']
        self.chassis = RigidChassis((chasDim['width'] / 1000.0, chasDim['length'] / 1000.0, chasDim['height'] / 1000.0))

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
