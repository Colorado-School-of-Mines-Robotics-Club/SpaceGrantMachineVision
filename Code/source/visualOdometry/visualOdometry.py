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


def makeStereoObjects():
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

    return leftStereo, rightStereo, wlsFilter
