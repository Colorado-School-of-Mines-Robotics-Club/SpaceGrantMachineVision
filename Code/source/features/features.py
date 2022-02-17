# Built in python libs
from typing import List, Tuple, Any

# Additional libs
import numpy as np
import cv2
from numba import jit

# Custom  imports
from source.logger.Logger import Logger
from source.utilities import exceptions
from source.cameras import DisplayManager
from .advancedFeatures import adaptiveRatioTest


# function that given to images computes their features
# this does not do any filtering
# takes two grayscale images and a cv2 feature detector
# @jit(forceobj=True)
def getImagePairKeyDesc(left: np.ndarray, right: np.ndarray, featureDetector: cv2.ORB) -> (List, np.ndarray, List,
                                                                                           np.ndarray):
    kp1, des1 = getImageKeyDesc(left, featureDetector)
    kp2, des2 = getImageKeyDesc(right, featureDetector)
    return kp1, des1, kp2, des2


# get features for a single image
# this does not do any filtering
# takes a single greyscale image
# @jit(forceobj=True)
def getImageKeyDesc(image: np.ndarray, featureDetector: cv2.ORB) -> (List, np.ndarray):
    return featureDetector.detectAndCompute(image, None)


# get point cordinates from a list of keypoints
def getPointsFromKeypoints(kp: List) -> np.ndarray:
    return cv2.KeyPoint_convert(kp)


# sorts matched keypoints returned directly from the cv2.matcher object
# this sorts them by distance
# @jit(forceobj=True)
def sortMatches(matches: List) -> np.ndarray:
    return np.array(sorted(matches, key=lambda x: x.distance))


# gets the image cordinates out of the matched keypoints
@jit(forceobj=True)
def getPointsFromMatches(matches: List, leftKp: List, rightKp: List) -> (List, List):
    return [leftKp[mat.queryIdx].pt for mat in matches], [rightKp[mat.trainIdx].pt for mat in matches]


# funtion that computes the matching features between two images and returns the corresponding points
# takes two grayscale images, a feature detector, and a matcher
# the showMatches optional parameter shows the total features and not the ones acquired through the ratio test
def computeMatchingPoints(left: np.ndarray, right: np.ndarray, featureDetector: cv2.ORB, featureMatcher, ratio=1.0,
                          featureRatio=0.1, stepSize=0.04, timeout=1000, show=False, threadedDisplay=True,
                          windowName="Matched Features") -> (List, List, List, np.ndarray, List, np.ndarray,
                                                             np.ndarray):
    try:
        leftKp, leftDesc, rightKp, rightDesc = getImagePairKeyDesc(left, right, featureDetector)
        if leftDesc is None or rightDesc is None:
            return [], [], leftKp, leftDesc, rightKp, rightDesc, list()
        matches = featureMatcher.match(leftDesc, rightDesc)
        # sort the matches
        sortedMatches = sortMatches(matches)
        # perform ratio test on matching key points
        ratioMatches = adaptiveRatioTest(sortedMatches, startingRatio=ratio, targetFeatureRatio=featureRatio,
                                         stepSize=stepSize, timeout=timeout)
        if len(ratioMatches) == 0:
            ratioMatches = sortedMatches
        # extract image coordinates of matches
        try:
            left_pts, right_pts = getPointsFromMatches(ratioMatches, leftKp, rightKp)
        except Exception:
            Logger.log("Warning: Could not pull points from features. No features?")
            return [], [], leftKp, leftDesc, rightKp, rightDesc, ratioMatches
            pass
        # show the output
        if show:
            try:
                matchedImg = cv2.drawMatches(left, leftKp, right, rightKp, ratioMatches, None, flags=2)
                if threadedDisplay:
                    DisplayManager.show(windowName, matchedImg)
                else:
                    cv2.imshow(windowName, matchedImg)
            except Exception:
                Logger.log("    computeMatchingPoints -> Failed to display matches")
                raise exceptions.FeatureDrawingError()
        return left_pts, right_pts, leftKp, leftDesc, rightKp, rightDesc, ratioMatches
    except Exception as e:  # generic exception catcher, just return no list of points
        raise e
