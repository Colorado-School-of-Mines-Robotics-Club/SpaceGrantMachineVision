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


@jit(forceobj=True)
def getSrcDstPointsFromMatches(matchedKp, prevKp, currKp):
    # (x, y) coordinates from the first image.
    prevPts = np.float32([prevKp[m.trainIdx].pt for m in matchedKp]).reshape(-1, 1, 2)
    # (x, y) coordinates from the second image.
    currPts = np.float32([currKp[m.trainIdx].pt for m in matchedKp]).reshape(-1, 1, 2)
    # converts to np.arrays
    return np.array(prevPts), np.array(currPts)


# actually gets average X
@jit(nopython=True)
def getAvgCoordinate(array):
    x_val_sum = 0
    for element in array:
        x_val_sum += element
    return x_val_sum / len(array)


@jit(forceobj=True)
def getCoordinateAverage(array):
    # Grabs only the first column (x values)
    avgX = getAvgCoordinate(array[:, 0][:, 0])
    # Grabs only the second column (y values)
    avgY = getAvgCoordinate(array[0, :][0, :])

    return avgX, avgY


@jit(forceobj=True)
def getTranslationXY(matchedKp: np.ndarray, prevKp: np.ndarray, currKp: np.ndarray) -> (float, float):
    prevPts, currPts = getSrcDstPointsFromMatches(matchedKp, prevKp, currKp)

    prevAvgX, prevAvgY = getCoordinateAverage(prevPts)
    currAvgX, currAvgY = getCoordinateAverage(currPts)

    transX = currAvgX - prevAvgX
    transY = currAvgY - prevAvgY

    return transX, transY


@jit(forceobj=True)
def getAvgTranslationXY(leftMatches: np.ndarray, prevLeftKp: np.ndarray, leftKp: np.ndarray, rightMatches: np.ndarray,
                        prevRightKp: np.ndarray, rightKp: np.ndarray) -> (float, float):
    leftX, leftY = getTranslationXY(leftMatches, prevLeftKp, leftKp)
    rightX, rightY = getTranslationXY(rightMatches, prevRightKp, rightKp)
    return (leftX + rightX) / 2, (leftY + rightY) / 2
