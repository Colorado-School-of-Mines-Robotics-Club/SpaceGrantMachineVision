# Built in python libs
import os

# Additional libs
import numpy as np
import cv2
from numba import jit
from multiprocessing import Queue, Process
import time
from typing import *

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


def PTcomputeDisparity(args: Tuple):
    queue = args[0]
    leftStereo = args[1]
    rightStereo = args[2]
    wlsFilter = args[3]
    show = args[4]
    td = args[5]

    left = queue.getInput()
    right = queue.getInput()

    if left is None or right is None:
        return np.zeros((640, 480))

    disparity = computeDisparity(leftStereo, rightStereo, wlsFilter, left, right, show=show, threadedDisplay=td)

    return disparity
