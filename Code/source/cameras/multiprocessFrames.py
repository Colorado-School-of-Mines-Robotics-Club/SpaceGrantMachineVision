from threading import Thread
from collections import deque
import cv2
import os
import time
import numpy as np

from typing import Union, Tuple
from enum import Enum
from openVO import StereoCamera


def PTframes(args: Tuple) -> Tuple[np.ndarray, np.ndarray]:
    queue, stereo, left_capture, right_capture = args

    got_left_frame = left_capture.grab()
    left_frame_time = time.perf_counter()

    got_right_frame = right_capture.grab()
    right_frame_time = time.perf_counter()

    if got_left_frame and got_right_frame:
        left_image = left_capture.retrieve()
        right_image = right_capture.retrieve()


    return (None, left_frame_time), (None, right_frame_time)
