# Built in python libs
import os
import time
from typing import Tuple, Union, Dict
import pickle

# Additional libs
import numpy as np
import cv2
from numba import jit
from openVO import StereoCamera

# Custom  imports
try:
    from source.logger.Logger import Logger
    from source.utilities import exceptions
    from .CaptureManager import CaptureManager, createCaptureSourceData
    from .DisplayManager import DisplayManager, createDisplaySourceData
    from .ThreadedCapture import ThreadedCapture
    from source.utilities.exceptions import CameraReadError
    from source.utilities.Config import Config
except ModuleNotFoundError:
    from Code.source.logger.Logger import Logger
    from Code.source.utilities import exceptions
    from .CaptureManager import CaptureManager, createCaptureSourceData
    from .DisplayManager import DisplayManager, createDisplaySourceData
    from .ThreadedCapture import ThreadedCapture
    from Code.source.utilities.exceptions import CameraReadError
    from Code.source.utilities.Config import Config


# gets the camera frames from the captureManager
def fetchCameraImages(leftSource: Union[str, int], rightSource: Union[str, int]) ->\
        Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    uncroppedLeft, left = CaptureManager.getFrame(leftSource)
    uncroppedRight, right = CaptureManager.getFrame(rightSource)
    return uncroppedLeft, uncroppedRight, left, right

# makes grayscale images of the bgr_images returned by readCameras
# @jit(forceobj=True)
def getGrayscaleImages(left: np.ndarray, right: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    return cv2.cvtColor(left, cv2.COLOR_BGR2GRAY), cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

# Function makes a window which displays both camera feeds next to each other
# Takes the images as two arguments: left, right images
# Has no return value
def showCameras(left: np.ndarray, right: np.ndarray, threadedDisplay=True):
    displayImg = np.concatenate((left, right), axis=1)
    if threadedDisplay:
        DisplayManager.show("Combined camera output", displayImg)
    else:
        cv2.imshow("Combined camera output", displayImg)


# gets the camera images from the capture manage
# undistorts and rectifies the images
# converts the images to grayscale
# shows the images
def fetchAndShowCameras(leftSource: Union[str, int], rightSource: Union[str, int], show=True, threadedDisplay=True) ->\
        Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    try:
        uncroppedLeft, uncroppedRight, left, right = fetchCameraImages(leftSource, rightSource)
        if left is None:
            raise CameraReadError(f"Port {leftSource}")
        if right is None:
            raise CameraReadError(f"Port {rightSource}")
        grayLeft, grayRight = getGrayscaleImages(left, right)
        if show:
            showCameras(uncroppedLeft, uncroppedRight, threadedDisplay)
        return uncroppedLeft, uncroppedRight, left, right, grayLeft, grayRight
    except Exception as e:
        raise e


# creates the cameras sources for ThreadedCapture and runs them into CaptureManager
def initCameras(leftCam: Union[str, int], rightCam: Union[str, int], stereo: StereoCamera, setExposure=False):
    # start CaptureManager for left and right cameras
    left = createCaptureSourceData(source=leftCam, type=ThreadedCapture.CameraType.LEFT, setExposure=setExposure)
    right = createCaptureSourceData(source=rightCam, type=ThreadedCapture.CameraType.RIGHT, setExposure=setExposure)
    CaptureManager.init([left, right], stereo)
    # wait for CaptureManager to start up threads
    while not CaptureManager.haveCamerasRead():
        time.sleep(0.05)
    # sleep time for cameras to read in a frame
    _, _, leftImage, rightImage = fetchCameraImages(leftCam, rightCam)
    while leftImage is None or rightImage is None:
        time.sleep(.1)
        _, _, leftImage, rightImage = fetchCameraImages(leftCam, rightCam)

# closes the camera sources
def closeCameras():
    CaptureManager.stopSources()

# loads all files from data that the robot needs
def loadCalibrationFiles(calibrationPath) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, Dict[str, np.ndarray]]:
    # one time file loading for the camera intrinsic matrices and undistortion coeff
    leftK = np.load(calibrationPath + "leftK.npy")
    rightK = np.load(calibrationPath + "rightK.npy")
    leftDistC = np.load(calibrationPath + "leftDistC.npy")
    rightDistC = np.load(calibrationPath + "rightDistC.npy")
    rectParams = pickle.load(open(calibrationPath + "rectParams.p", "rb"))

    return leftK, rightK, leftDistC, rightDistC, rectParams
