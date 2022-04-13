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

    pass
