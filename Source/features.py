# Built in python libs
import os
import sys
import time

# Additional libs
import numpy as np
import cv2
from numba import jit

# Custom  imports
from logger import Logger
import exceptions
import utility


# function that given to images computes their features
# this does not do any filtering
# takes two grayscale images and a cv2 feature detector
def getImagePairKeyDesc(left, right, featureDetector):
    kp1, des1 = featureDetector.detectAndCompute(left, None)
    kp2, des2 = featureDetector.detectAndCompute(right, None)
    return kp1, des1, kp2, des2

# get features for a single image
# this does not do any filtering
# takes a single greyscale image
def getImageKeyDesc(image, featureDetector):
    try:
        return featureDetector.detectAndCompute(image, None)
    except:
        raise exceptions.FeatureMatchingError("Error running detect and compute in GetImageKeyDesc")

# get point cordinates from a list of keypoints
def getPointsFromKeypoints(kp):
    try:
        return cv2.KeyPoint_convert(kp)
    except:
        raise exceptions.FeatureMatchingError("Error converting array cv2.KeyPoint to array of array points")


# TODO
# add comments

# ALL FUNCTIONS BELOW USED IN COMPUTE MATCHING POINTS
# =================================================================================
# sorts matched keypoints returned directly from the cv2.matcher object
# this sorts them by distance
def sortMatches(matches):
    return np.array(sorted(matches, key=lambda x: x.distance))

# gets the image cordinates out of the matched keypoints
def getPointsFromMatches(matches, leftKp, rightKp):
    return [leftKp[mat.queryIdx].pt for mat in matches], [rightKp[mat.trainIdx].pt for mat in matches]

# performs the ratio test on a set of matched keypoints
# the ratio test filters matched keypoints out if they are greater than the minimum seperation between matched
# by a set amount. The usual value is 3. Thus, if the distance between two matched features is 3 times larger than the
# minimum pixel distance between 2 matched features, than we can say it is invalid and in outlier.
# This is because the typical image to image comparision will have a small change between frames
# takes a list of keypoints, a minimum distance, and a given ratio
# kpMatches must be SORTED for optimization purposes
def ratioTest(kpMatches, ratio):
    try:
        minDist = kpMatches[0].distance
    except:
        raise exceptions.FeatureMatchingError("ratioTest: No matched feature points present in kpMatches")
    goodDistanceDiffs = []
    for m in kpMatches:
        if m.distance < ratio * minDist:
            goodDistanceDiffs.append(m)
        else:
            break
    return sorted(goodDistanceDiffs, key=lambda x: x.distance)

# funtion that computes the matching features between two images and returns the corresponding points
# takes two grayscale images, a feature detector, and a matcher
def computeMatchingPoints(left, right, featureDetector, featureMatcher, ratio=3.0, showMatches=False, verbose=False):
    if verbose:
        Logger.log("computeMatchingPoints running...")
    try:
        if verbose:
            Logger.log("    -> Generating keypoints and descriptors then matching keypoints")
        leftKp, leftDesc, rightKp, rightDesc = getImagePairKeyDesc(left, right, featureDetector)
        matches = featureMatcher.match(leftDesc, rightDesc)
        if len(matches) == 0:
            raise exceptions.FeatureMatchingError("computeMatchingPoints: No matched features present in the matched features ")
        if verbose:
            Logger.log("          DONE")
            Logger.log("    -> Sorting matches and performing the ratio test")
        # sort the matches
        sortedMatches = sortMatches(matches)
        # perform ratio test on matching keypoints
        ratioMatches = ratioTest(sortedMatches, ratio=ratio)
        if verbose:
            Logger.log("          DONE")
            Logger.log("    -> Separating image points from matches")
        # extract image cordinates of matches
        left_pts, right_pts = getPointsFromMatches(ratioMatches, leftKp, rightKp)
        # show the output
        if verbose:
            Logger.log("          DONE")
        if showMatches:
            matchedImg = cv2.drawMatches(left, left_pts, right, right_pts, ratioMatches, None)
            cv2.imshow("Matched Features", matchedImg)
        return left_pts, right_pts
    except Exception as e: # generic exception catcher, just return no list of points
        Logger.log("Generating an exception inside of computeMatchingPoints")
        Logger.log(e)
        return [], []
