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


def detectHorizonLine(image, show=True):
    # use edge/contour detector to find the horizon line in an image

    # returns a line across the middle of the screen
    # PLACEHOLDER
    horizonLine = [0, image.shape[1] / 2], [image.shape[0], image.shape[1] / 2]

    if show:
        lineImage = np.copy(image)
        cv2.line(lineImage, (horizonLine[0][0], horizonLine[0][1]), (horizonLine[1][0], horizonLine[1][1]), (0, 0, 255), 2)
        cv2.imshow("Horizon Line", lineImage)

    return horizonLine

def detect_horizon_line(image_grayscaled):
    """Detect the horizon's starting and ending points in the given image
    The horizon line is detected by applying Otsu's threshold method to
    separate the sky from the remainder of the image.
    :param image_grayscaled: grayscaled image to detect the horizon on, of
     shape (height, width)
    :type image_grayscale: np.ndarray of dtype uint8
    :return: the (x1, x2, y1, y2) coordinates for the starting and ending
     points of the detected horizon line
    :rtype: tuple(int)
    """

    msg = ('`image_grayscaled` should be a grayscale, 2-dimensional image '
           'of shape (height, width).')
    assert image_grayscaled.ndim == 2, msg
    image_blurred = cv2.GaussianBlur(image_grayscaled, ksize=(3, 3), sigmaX=0)

    _, image_thresholded = cv2.threshold(
        image_blurred, thresh=0, maxval=1,
        type=cv2.THRESH_BINARY+cv2.THRESH_OTSU
    )
    image_thresholded = image_thresholded - 1
    image_closed = cv2.morphologyEx(image_thresholded, cv2.MORPH_CLOSE,
                                    kernel=np.ones((9, 9), np.uint8))

    horizon_x1 = 0
    horizon_x2 = image_grayscaled.shape[1] - 1
    horizon_y1 = max(np.where(image_closed[:, horizon_x1] == 0)[0])
    horizon_y2 = max(np.where(image_closed[:, horizon_x2] == 0)[0])

    return horizon_x1, horizon_x2, horizon_y1, horizon_y2


def filterBoundingBoxesByHorizon(image, boundingBoxes, horizonLine):
    filteredBoundingBoxes = boundingBoxes
    for i in range(len(filteredBoundingBoxes)):
        if filteredBoundingBoxes[i][1][1] > horizonLine[0][1]:
            np.delete(filteredBoundingBoxes, i)
        # yValue = boundingBoxes[i][1][1]
        if filteredBoundingBoxes[i][0][1] > horizonLine[0][1] and filteredBoundingBoxes[i][1][1] < horizonLine[0][1]:
            np.delete(filteredBoundingBoxes, i)
        #if yValue > horizonLine
    return filteredBoundingBoxes
