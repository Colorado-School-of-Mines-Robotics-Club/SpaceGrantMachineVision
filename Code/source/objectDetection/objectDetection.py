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
from .horizonDetection import detectHorizonLine, filterBoundingBoxesByHorizon, cropBoundingBoxesByHorizon


def objectDetection(image: np.ndarray, featurePts: np.ndarray, binSize=30.0, featuresPerPixel=0.03, show=True,
                    threadedDisplay=False) -> List:
    # run the contour detection
    contourBoundingBoxes = getContourBoundingBoxes(image, show=show, threadedDisplay=threadedDisplay)

    # run feature density calculations
    featureDenseBoundingBoxes = findFeatureDenseBoundingBoxes(image, featurePts, binSize=binSize,
                                                              featuresPerPixel=featuresPerPixel, show=show,
                                                              threadedDisplay=threadedDisplay)

    # right now just combine the boundingBoxes, in the future should make some decisions on them
    objectBoundingBoxes = findObjects(contourBoundingBoxes, featureDenseBoundingBoxes)

    # Filter bounding boxes by horizon
    horizonLine = detectHorizonLine(image, show=show)
    filteredObjectBoundingBoxes = filterBoundingBoxesByHorizon(image, objectBoundingBoxes, horizonLine, show=show,
                                                               threadedDisplay=threadedDisplay)


    return filteredObjectBoundingBoxes


def findObjects(contourBoxes: List, featureDenseBoxes: List) -> List:
    return contourBoxes + featureDenseBoxes


def compile_object_detection() -> None:
    points = np.random.randint(100, 200, (200, 1)).astype('uint64')

    dummy_image = cv2.imread("../Data/Calibration/LeftCaptures/10.png")

    _ = findFeatureDenseBoundingBoxes(dummy_image, points, binSize=10, featuresPerPixel=0.01, show=False)
