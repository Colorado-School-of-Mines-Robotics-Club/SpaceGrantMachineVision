# attempt to import packages upwards, if it fails for circular import then just ignore it
try:
    from cameras import *
except ImportError:
    pass
try:
    from features import *
except ImportError:
    pass
try:
    from hardware import *
except ImportError:
    pass
try:
    from kinematics import *
except ImportError:
    pass
try:
    from logger import *
except ImportError:
    pass
try:
    from objectDetection import *
except ImportError:
    pass
try:
    from pathfinding import *
except ImportError:
    pass
try:
    from simulation import *
except ImportError:
    pass
try:
    from utilities import *
except ImportError:
    pass
try:
    from visualOdometry import *
except ImportError:
    pass
# This should just compile everything I suppose
