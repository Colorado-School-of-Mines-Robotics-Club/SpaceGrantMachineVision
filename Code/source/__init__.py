import numpy as np
import cv2
from .features import compile_features
from .kinematics import compile_kinematics
from .objectDetection import compile_object_detection


compile_features()
compile_kinematics()
compile_object_detection()
