try:
    import source.cameras as cameras
    import source.concurrency as concurrency
    import source.features as features
    import source.hardware as hardware
    import source.kinematics as kinematics
    import source.logger as logger
    import source.objectDetection as objectDetection
    import source.pathfinding as pathfinding
    import source.simulation as simulation
    import source.utilities as utilities
    import source.visualOdometry as visualOdometry
except ModuleNotFoundError:
    import Code.source.cameras as cameras
    import Code.source.concurrency as concurrency
    import Code.source.features as features
    import Code.source.hardware as hardware
    import Code.source.kinematics as kinematics
    import Code.source.logger as logger
    import Code.source.objectDetection as objectDetection
    import Code.source.pathfinding as pathfinding
    import Code.source.simulation as simulation
    import Code.source.utilities as utilities
    import Code.source.visualOdometry as visualOdometry
