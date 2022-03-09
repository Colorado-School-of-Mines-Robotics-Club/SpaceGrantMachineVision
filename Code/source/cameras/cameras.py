# Built in python libs
import os
import time
from typing import Tuple, Union

# Additional libs
import numpy as np
import cv2
from numba import jit

# Custom  imports
from source.logger.Logger import Logger
from source.utilities import exceptions
from .CaptureManager import CaptureManager, createCaptureSourceData
from .DisplayManager import DisplayManager, createDisplaySourceData
from .ThreadedCapture import ThreadedCapture
from source.utilities.exceptions import CameraReadError
from source.utilities.Config import Config


# gets the camera frames from the captureManager
def fetchCameraImages(leftSource: Union[str, int], rightSource: Union[str, int]) -> Tuple[np.ndarray, np.ndarray]:
    left, right = CaptureManager.getFrame(leftSource), CaptureManager.getFrame(rightSource)
    if left.shape != right.shape:
        minHeight = min(left.shape[0], right.shape[0])
        minWidth = min(left.shape[1], right.shape[1])
        newDim = (minWidth, minHeight)
        leftResize = cv2.resize(left, newDim)
        rightResize = cv2.resize(right, newDim)
        return leftResize, rightResize
    return left, right


# makes grayscale images of the bgr_images returned by readCameras
# @jit(forceobj=True)
def getGrayscaleImages(left: np.ndarray, right: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    return cv2.cvtColor(left, cv2.COLOR_BGR2GRAY), cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)


# Function makes a window which displays both camera feeds next to each other
# Takes the images as two arguments: left, right images
# Has no return value
@jit(forceobj=True)
def showCameras(left: np.ndarray, right: np.ndarray, threadedDisplay=True):
    displayImg = np.concatenate((left, right), axis=1)
    if threadedDisplay:
        DisplayManager.show("Combined camera output", displayImg)
    else:
        cv2.imshow("Combined camera output", displayImg)


# gets the camera images from the capture manager
# converts the images to grayscale
# shows the images
def fetchAndShowCameras(leftSource: Union[str, int], rightSource: Union[str, int], show=True, threadedDisplay=True) ->\
        Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    try:
        left, right = fetchCameraImages(leftSource, rightSource)
        if left is None:
            raise CameraReadError(f"Port {leftSource}")
        if right is None:
            raise CameraReadError(f"Port {rightSource}")
        grayLeft, grayRight = getGrayscaleImages(left, right)
        if show:
            showCameras(left, right, threadedDisplay)
        return left, right, grayLeft, grayRight
    except Exception as e:
        raise e


# creates the cameras sources for ThreadedCapture and runs them into CaptureManager
def initCameras(leftCam: Union[str, int], rightCam: Union[str, int], setExposure=False):
    leftK, rightK, leftDistC, rightDistC = loadUndistortionFiles()
    # start CaptureManager for left and right cameras
    left = createCaptureSourceData(source=leftCam, K=leftK, distC=leftDistC, setExposure=setExposure)
    right = createCaptureSourceData(source=rightCam, K=rightK, distC=rightDistC, setExposure=setExposure)
    CaptureManager.init([left, right])
    # wait for CaptureManager to start up threads
    while not CaptureManager.haveCamerasRead():
        time.sleep(0.05)
    # sleep time for cameras to read in a frame
    leftImage, rightImage = fetchCameraImages(leftCam, rightCam)
    while leftImage is None or rightImage is None:
        time.sleep(.1)
        leftImage, rightImage = fetchCameraImages(leftCam, rightCam)


# closes the camera sources
def closeCameras():
    CaptureManager.stopSources()


# loads all files from data that the robot needs
def loadUndistortionFiles() -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    # one time file loading for the camera intrinsic matrices and undistortion coeff
    Config.init()
    calibrationPath = Config.getFilepathsDict()["calibrationPath"]
    if not os.path.isdir(calibrationPath):
        calibrationPath = "../" + calibrationPath
    leftK = np.load(calibrationPath + "leftK.npy")
    rightK = np.load(calibrationPath + "rightK.npy")
    leftDistC = np.load(calibrationPath + "leftDistC.npy")
    rightDistC = np.load(calibrationPath + "rightDistC.npy")

    return leftK, rightK, leftDistC, rightDistC
