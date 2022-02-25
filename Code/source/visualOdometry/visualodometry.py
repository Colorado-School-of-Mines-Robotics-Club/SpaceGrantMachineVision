# Built in python libs
import os

# Additional libs
import numpy as np
import cv2
from numba import jit
from multiprocessing import Queue, Process
import time

# Custom  imports
from source.cameras import DisplayManager
from source.utilities import Config


# compute the disparity map of the two grayscale images given
# takes a stereo matcher object and two grayscale images
# @jit(forceobj=True)
def computeDisparity(leftStereo: cv2.StereoSGBM, rightStereo: cv2.StereoMatcher,
                     wlsFilter: cv2.ximgproc_DisparityWLSFilter, left: np.ndarray, right: np.ndarray, show=False,
                     threadedDisplay=False):
    left_disp = leftStereo.compute(left, right)
    right_disp = rightStereo.compute(right, left)
    filtered_disp = wlsFilter.filter(left_disp, left, disparity_map_right=right_disp)
    disparity = cv2.normalize(filtered_disp, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    if show:
        if threadedDisplay:
            DisplayManager.show("Disparity map", disparity)
        else:
            cv2.imshow("Disparity map", disparity)
    return disparity


def disparityProcess(imageQueue: Queue, mapQueue: Queue, show=False, threadedDisplay=False):
    Config.init()
    sbgmPs = Config.getSBGMParamsDict()
    wlsParams = Config.getWLSParamsDict()
    leftStereo = cv2.StereoSGBM_create(minDisparity=sbgmPs['minDisparity'], numDisparities=sbgmPs['numDisparities'],
                                       blockSize=sbgmPs['blockSize'], P1=sbgmPs['P1'], P2=sbgmPs['P2'],
                                       disp12MaxDiff=sbgmPs['disp12MaxDiff'], preFilterCap=sbgmPs['preFilterCap'],
                                       uniquenessRatio=sbgmPs['uniquenessRatio'],
                                       speckleWindowSize=sbgmPs['speckleWindowSize'],
                                       speckleRange=sbgmPs['speckleRange'])
    rightStereo = cv2.ximgproc.createRightMatcher(leftStereo)
    wlsFilter = cv2.ximgproc.createDisparityWLSFilter(leftStereo)
    wlsFilter.setLambda(wlsParams['lambda'])
    wlsFilter.setSigmaColor(wlsParams['sigma'])

    while True:
        left = imageQueue.get()
        right = imageQueue.get()
        disparity = computeDisparity(leftStereo, rightStereo, wlsFilter, left, right)
        mapQueue.put(disparity)
        if show:
            if threadedDisplay:
                DisplayManager.show("Disparity", disparity)
            else:
                cv2.imshow("Disparity", disparity)
        cv2.waitKey(1)


def startDisparityProcess(imageQueue: Queue, mapQueue: Queue, show=False, threadedDisplay=False) -> Process:
    p = Process(target=disparityProcess, args=(imageQueue, mapQueue, show, threadedDisplay,), daemon=True)
    p.start()
    return p
