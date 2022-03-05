from source.features import compile_features
from source.kinematics import compile_kinematics
from source.objectDetection import compile_object_detection


def jit_compile_all():
    compile_features()
    compile_kinematics()
    compile_object_detection()
