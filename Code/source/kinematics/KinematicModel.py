from typing import Tuple, List, Union

try:
    from .WheelAssembly import WheelAssembly
    from .elements.RigidChassis import RigidChassis
    from source.utilities import Config
except ModuleNotFoundError:
    from .WheelAssembly import WheelAssembly
    from .elements.RigidChassis import RigidChassis
    from Code.source.utilities import Config


class KinematicModel:
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

    # suspensionHeightTargets units are in centimeters and then get converted to meters
    def update(self, wheelVelocities: List[Union[float, None]] = None, swerveAngles: List[Union[float, None]] = None,
               suspensionHeightTargets: List[Union[float, None]] = None):
        if suspensionHeightTargets is not None:
            suspensionHeightTargets = [target / 100.0 for target in suspensionHeightTargets]
        for idx, wheel in enumerate(self.wheels):
            input1 = swerveAngles[idx] if swerveAngles is not None else None
            input2 = wheelVelocities[idx] if wheelVelocities is not None else None
            input3 = suspensionHeightTargets[idx] if suspensionHeightTargets is not None else None
            wheel.update(swerveAngle=input1, wheelVelocity=input2,
                         suspensionTarget=input3)
        # TODO, not urgent
        # make a determination of the velocity from wheels and/or other math.
        self.chassis.update()
