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
def computeDisparity(stereo, left, right, show=False, threadedDisplay=True):
    disparity = stereo.compute(left, right).astype(np.float32)
    disparity = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    if show:
        if threadedDisplay:
            DisplayManager.show("Disparity map", disparity)
        else:
            cv2.imshow("Disparity map", disparity)
    return disparity
