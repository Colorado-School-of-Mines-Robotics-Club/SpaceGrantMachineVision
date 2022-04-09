from Code.source.hardware.KinematicHardwareInterface import KinematicHardwareInterface

interface = KinematicHardwareInterface()

# interface.kinematicModel.update()
interface.kinematicModel.update(wheelVelocities=[1.0, 1.0, 1.0, 1.0], swerveAngles=[1.0, 1.0, 1.0, 1.0],
                                suspensionHeightTargets=[5.0, 1.0, 1.0, 1.0])
print(interface.kinematicModel.wheels[0].suspension.getAngle())

for i in range(20):
    interface.kinematicModel.update(wheelVelocities=[i, i, i, i], swerveAngles=[i, i, i, i],
                                    suspensionHeightTargets=[i, i, i, i])
    print(f"Height: {i}, angle:{interface.kinematicModel.wheels[0].suspension.getAngle()}")
