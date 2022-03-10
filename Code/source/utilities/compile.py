from source.features import compile_features
from source.kinematics import compile_kinematics
from source.objectDetection import compile_object_detection

from numba.core.errors import NumbaDeprecationWarning, NumbaPendingDeprecationWarning
import warnings


def suppress_numba_warnings() -> None:
    warnings.simplefilter('ignore', category=NumbaDeprecationWarning)
    warnings.simplefilter('ignore', category=NumbaPendingDeprecationWarning)


def jit_compile_all(verbose=False) -> None:
    suppress_numba_warnings()
    if verbose:
        print("Compiling all...")
        print("    Compiling features...")
    compile_features()
    if verbose:
        print("        ...DONE")
        print("    Compiling kinematics...")
    compile_kinematics()
    if verbose:
        print("        ...DONE")
        print("    Compiling object detection...")
    compile_object_detection()
    if verbose:
        print("        ...DONE")
        print("  ...DONE")
