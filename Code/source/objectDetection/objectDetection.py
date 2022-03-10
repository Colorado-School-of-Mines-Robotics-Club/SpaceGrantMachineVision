# Built in python libs
import os
import sys
import time
import math
from typing import List

# Additional libs
import numpy as np
import cv2
from numba import jit, njit

# Custom  imports
from source.logger.Logger import Logger
from source.utilities import exceptions
from source.utilities.boundingBoxes import drawBoundingBoxes, cv2RectToNpBoxes
from source.features.features import getPointsFromKeypoints, getImageKeyDesc, getImagePairKeyDesc
from .featureDensity import findFeatureDenseBoundingBoxes
from .contourDetection import getContourBoundingBoxes


def objectDetection(image: np.ndarray, featurePts: np.ndarray, binSize=30.0, featuresPerPixel=0.03, show=True,
                    threadedDisplay=False) -> List:
    # run the contour detection
    contourBoundingBoxes = getContourBoundingBoxes(image, show=show, threadedDisplay=threadedDisplay)

    # run feature density calculations
    featureDenseBoundingBoxes = findFeatureDenseBoundingBoxes(image, featurePts, binSize=binSize,
                                                              featuresPerPixel=featuresPerPixel, show=show,
                                                              threadedDisplay=threadedDisplay)

    # right now just combine the boundingBoxes, in the future should make some decisions on them
    objectBoundingBoxes = contourBoundingBoxes + featureDenseBoundingBoxes

    return objectBoundingBoxes


def compile_object_detection():
    points = np.random.randint(100, 200, (20, 1)).astype('uint64')

    dummy_image = np.random.randint(0, 255, (640, 480)).astype('uint8')

    _ = findFeatureDenseBoundingBoxes(dummy_image, points)
