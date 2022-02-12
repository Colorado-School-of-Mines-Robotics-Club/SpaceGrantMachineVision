# Built in python libs
import os
import sys
import time
import math

# Additional libs
import numpy as np
import cv2
from numba import jit, njit

# Custom  imports
from Code.source.logger.Logger import Logger
from Code.source.utilities import exceptions
from Code.source.utilities.boundingBoxes import drawBoundingBoxes
from Code.source.features.features import getPointsFromKeypoints, getImageKeyDesc, getImagePairKeyDesc
from .featureDensity import findFeatureDenseBoundingBoxes


def detectHorizonLine(image, show=False):
    # use edge/contour detector to find the horizon line in an image

    # returns a line across the middle of the screen
    # PLACEHOLDER
    horizonLine = [0, image.shape[1] / 2], [image.shape[0], image.shape[1] / 2]

    if show:
        lineImage = np.copy(image)
        cv2.line(lineImage, (horizonLine[0][0], horizonLine[0][1]), (horizonLine[1][0], horizonLine[1][1]), (0, 0, 255), 2)
        cv2.imshow("Horizon Line", lineImage)

    return horizonLine

def filterBoundingBoxesByHorizon(image, boundingBoxes, horizonLine):

    return boundingBoxes


