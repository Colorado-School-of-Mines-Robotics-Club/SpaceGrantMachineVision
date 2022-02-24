# Built in python libs
import os

# Additional libs
import numpy as np
import cv2
from numba import jit

# Custom  imports
from source.cameras import DisplayManager


# compute the disparity map of the two grayscale images given
# takes a stereo matcher object and two grayscale images
# @jit(forceobj=True)
def computeDisparity(leftStereo, rightStereo, wlsFilter, left, right, show=False, threadedDisplay=True):
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
