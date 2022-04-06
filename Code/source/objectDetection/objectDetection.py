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
from source.utilities.boundingBoxes import drawBoundingBoxes, cv2RectToNpBoxes, determineConnection,\
    getBoundingBoxArea, simplifyBoundingBoxes
from source.features.features import getPointsFromKeypoints, getImageKeyDesc, getImagePairKeyDesc
from .featureDensity import findFeatureDenseBoundingBoxes
from .contourDetection import getContourBoundingBoxes
from .horizonDetection import detectHorizonLine, filterBoundingBoxesByHorizon, cropBoundingBoxesByHorizon

from . import experimental


def objectDetection(image: np.ndarray, featurePts: np.ndarray, binSize=30.0, featuresPerPixel=0.03,
                    percentAreaThreshold=0.025, connectedFeaturesThresh=10, simplifyFinalOutput=True, show=True,
                    threadedDisplay=False) -> List:
    # run the contour detection
    contourBoundingBoxes = getContourBoundingBoxes(image, show=show, threadedDisplay=threadedDisplay)

    # run feature density calculations
    featureDenseBoundingBoxes = findFeatureDenseBoundingBoxes(image, featurePts, binSize=binSize,
                                                              featuresPerPixel=featuresPerPixel, show=show,
                                                              threadedDisplay=threadedDisplay)

    # right now just combine the boundingBoxes, in the future should make some decisions on them
    objectBoundingBoxes = findObjects(image, contourBoundingBoxes, featureDenseBoundingBoxes, binSize,
                                      percentAreaThreshold, connectedFeaturesThresh, simplify=simplifyFinalOutput)

    if show:
        drawBoundingBoxes(image, objectBoundingBoxes, windowName="Objects", show=True,
                          threadedDisplay=threadedDisplay)

    # Filter bounding boxes by horizon
    horizonLine = detectHorizonLine(image, show=show)
    filteredObjectBoundingBoxes = filterBoundingBoxesByHorizon(image, objectBoundingBoxes, horizonLine, show=show,
                                                               threadedDisplay=threadedDisplay)

    # testing for kmeans
    _ = experimental.segmentColors(image, method='kmeans', K=3, iterations=10, downscale=True, downscaleMethod='linear',
                                   downscaleRatio=0.4, show=show, threadedDisplay=threadedDisplay)

    return filteredObjectBoundingBoxes


def findObjects(image: np.ndarray, contourBoxes: List, featureDenseBoxes: List, binSize: float,
                percentAreaThreshold: float, connectedFeaturesThresh: int, simplify=True) -> List:
    objectsByArea = findObjectsByArea(image, contourBoxes, featureDenseBoxes, binSize, percentAreaThreshold,
                                      connectedFeaturesThresh)
    objectBoxes = objectsByArea

    if simplify:
        return simplifyBoundingBoxes(objectBoxes)
    return objectBoxes


def findObjectsByArea(image: np.ndarray, contourBoxes: List, featureDenseBoxes: List, binSize: float,
                      percentAreaThresh=0.025, connectedFeaturesThresh=10) -> List:
    objectBoxes = list()
    h, w, _ = image.shape
    imageArea = float(h * w)

    for contourBox in contourBoxes:
        internalArea = 0
        for featureBox in featureDenseBoxes:
            if determineConnection(contourBox, featureBox):
                internalArea += getBoundingBoxArea(featureBox)
        if (float(internalArea) / imageArea) > percentAreaThresh:
            # for now only use the contour boxes as objects
            objectBoxes.append(contourBox)

    for featureBox in featureDenseBoxes:
        if getBoundingBoxArea(featureBox) > binSize ** 2 * connectedFeaturesThresh:
            objectBoxes.append(featureBox)

    return objectBoxes


def compile_object_detection() -> None:
    points = np.random.randint(100, 200, (200, 1)).astype('uint64')

    dummy_image = cv2.imread("../Data/Calibration/LeftCaptures/10.png")

    _ = findFeatureDenseBoundingBoxes(dummy_image, points, binSize=10, featuresPerPixel=0.01, show=False)
