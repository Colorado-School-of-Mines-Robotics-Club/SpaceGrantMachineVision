from threading import Thread
from collections import deque
import cv2
import os
import time
import numpy as np

from typing import Union, Tuple
from enum import Enum
from openVO import StereoCamera

from .DisplayManager import DisplayManager


def safeOpenCaptureSource(source: Union[str, int]) -> cv2.VideoCapture:
    try:
        return cv2.VideoCapture(source)
    except Exception:
        raise Exception(f'Error defining cv2.videoCapture object for source: {source}')


def waitForFrame(stream: cv2.VideoCapture) -> None:
    startTime = time.time()
    while True:
        got_frame, temp_frame = stream.read()
        if got_frame:
            return
        if time.time() - startTime > 60:
            raise Exception("Timed out in camera read")
        time.sleep(0.1)


def frameStaticBuilder(args: Tuple) -> Tuple:
    leftSource, rightSource, leftK, rightK, leftDistC, rightDistC, rectParams, sgbmPs, frameSize = args

    stereo = StereoCamera(leftK, leftDistC, rightK, rightDistC, rectParams, sgbmPs, frameSize)

    leftCapture = safeOpenCaptureSource(leftSource)
    rightCapture = safeOpenCaptureSource(rightSource)

    waitForFrame(leftCapture)
    waitForFrame(rightCapture)

    return stereo, leftCapture, rightCapture


def PTframes(args: Tuple) -> Union[Tuple[Tuple[np.array, np.array, float], Tuple[np.array, np.array, float]], None]:
    queue, stereo, left_capture, right_capture, threshold, show, td = args

    got_left_frame = left_capture.grab()
    left_frame_time = time.perf_counter()

    got_right_frame = right_capture.grab()
    right_frame_time = time.perf_counter()

    if right_frame_time - left_frame_time >= threshold:
        return None

    if not got_left_frame or not got_right_frame:
        return None

    left_image = left_capture.retrieve()[1]
    right_image = right_capture.retrieve()[1]
    # left_image = left_capture.read()
    # right_image = right_capture.read()

    left_frame = stereo.undistort_rectify_left(left_image)
    left_frame_cropped = stereo.crop_to_valid_region_left(left_frame)

    right_frame = stereo.undistort_rectify_right(right_image)
    right_frame_cropped = stereo.crop_to_valid_region_right(right_frame)

    if show:
        combinedImage = np.concatenate((left_frame, right_frame), axis=1)
        if td:
            DisplayManager.show("Camera Input", combinedImage)
        else:
            cv2.imshow("Camera Input", combinedImage)
        cv2.waitKey(1)

    return (left_frame, left_frame_cropped, left_frame_time), (right_frame, right_frame_cropped, right_frame_time)
