import cv2
import time
from typing import *


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


def getFrames(leftsource, rightsource):
    left_capture = cv2.VideoCapture(leftsource)
    right_capture = cv2.VideoCapture(rightsource)
    #
    # got_left_frame = left_capture.grab()
    # left_frame_time = time.perf_counter()
    #
    # got_right_frame = right_capture.grab()
    # right_frame_time = time.perf_counter()
    left_image = left_capture.read()
    right_image = right_capture.read()

    print("got the frames")

    # if right_frame_time - left_frame_time >= 0.5:
    #     return None
    # if not got_left_frame or not got_right_frame:
    #     return None
    #
    # left_image = left_capture.retrieve()[1]
    # right_image = right_capture.retrieve()[1]

    return left_image, right_image


while True:
    _, _ = getFrames(0, 2)
