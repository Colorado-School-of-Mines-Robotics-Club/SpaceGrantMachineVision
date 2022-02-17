# Built in python libs
from typing import List, Tuple, Any

# Additional libs
import numpy as np
import cv2
from numba import jit, njit, prange

# Custom  imports
from source.logger.Logger import Logger
from source.utilities import exceptions
from source.cameras import DisplayManager


# performs the ratio test on a set of matched keypoints
# the ratio test filters matched keypoints out if they are greater than the minimum seperation between matched
# by a set amount. The usual value is 3. Thus, if the distance between two matched features is 3 times larger than the
# minimum pixel distance between 2 matched features, than we can say it is invalid and in outlier.
# This is because the typical image to image comparision will have a small change between frames
# takes a list of keypoints, a minimum distance, and a given ratio
# kpMatches must be SORTED for optimization purposes
@jit(forceobj=True)
def ratioTest(kpMatches: np.ndarray, ratio: float) -> List:
    if len(kpMatches) > 0:
        minDist = kpMatches[0].distance
    else:
        minDist = 0.0
    goodDistanceDiffs = []
    for m in kpMatches:
        if m.distance < ratio * minDist:
            goodDistanceDiffs.append(m)
        else:
            break
    return goodDistanceDiffs


@jit(forceobj=True)
def adaptiveRatioTest(kpMatches: np.ndarray, startingRatio: float, targetFeatureRatio: float, stepSize: float) -> List:
    ratioPoints: List = list()
    currentFeatureRatio = 0.0
    counter = 0
    while currentFeatureRatio < targetFeatureRatio:
        ratioPoints = ratioTest(kpMatches, startingRatio + stepSize * counter)
        currentFeatureRatio = len(ratioPoints) / len(kpMatches)
        counter += 1
    # print(f"CurrentFeatureRatio: {currentFeatureRatio} with ratio: {startingRatio + (counter - 1) * stepSize}")
    return ratioPoints

# actually gets average X
@njit(fastmath=True)
def getTranslationX(array_x):
    x_val_sum = 0
    for element in array_x:
        x_val_sum += element
    return x_val_sum / len(array_x)

# actually gets average Y
@njit(fastmath=True)
def getTranslationY(array_y):
    y_val_sum = 0
    for element in array_y:
        y_val_sum += element
    return y_val_sum / len(array_y)


def getTranslationXY(matchedKp: np.ndarray, prevKp: np.ndarray, currKp: np.ndarray) -> (float, float):
    # (x, y) coordinates from the first image.
    dst_pts = np.float32([prevKp[m.trainIdx].pt for m in matchedKp]).reshape(-1, 1, 2)
    # (x, y) coordinates from the second image.
    src_pts = np.float32([currKp[m.trainIdx].pt for m in matchedKp]).reshape(-1, 1, 2)

    dst_pts = np.array(dst_pts)
    src_pts = np.array(src_pts)

    # gets average x,y cordinates CURRENTLY
    # Grabs only the first column (x values)
    dst_pts_x_translation = getTranslationX(dst_pts[:, 0][:, 0])
    # Grabs only the second column (y values)
    dst_pts_y_translation = getTranslationY(dst_pts[0, :][0, :])
    # Apply the same process to the belows
    src_pts_x_translation = getTranslationX(src_pts[:, 0][:, 0])
    src_pts_y_translation = getTranslationY(src_pts[0, :][0, :])

    return 'mock and stubs'

