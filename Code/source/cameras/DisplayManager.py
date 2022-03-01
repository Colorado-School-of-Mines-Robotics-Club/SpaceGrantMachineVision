from typing import List, Dict, Union
import numpy as np
import cv2
from cv2 import error as cv2Error
import time

from .ThreadedDisplay import ThreadedDisplay


def createDisplaySourceData(windowName="Output", frame=None) -> List:
    return [windowName, frame]


class DisplayManager:
    """
        Class that managers threadedDisplay objects
    """
    # generate a dict indexed by source to aggregate the threads
    displays = dict()

    # inits the DisplayMananger
    @classmethod
    def init(cls):
        cls.displays = dict()

    # sources is a list of lists formed where each entry should be created by createSourceData
    @classmethod
    def showGroup(cls, displays: List):
        # initialize and start all threads
        for display in displays:
            cls.displays[display[0]] = ThreadedDisplay(display[0], display[1]).start()

    # gets the frame from a specific source
    @classmethod
    def show(cls, windowName: str, frame: np.ndarray):
        if windowName in cls.displays:
            cls.displays[windowName].update(frame)
        else:
            cls.displays[windowName] = ThreadedDisplay(windowName, frame).start()

    @classmethod
    def updateFPS(cls, windowName: str, fps: float):
        cls.displays[windowName].updateFPS(fps)

    @classmethod
    def updateAllFPS(cls, fps: float):
        for source, thread in cls.displays.items():
            cls.updateFPS(source, fps)

    # stops a threadedDisplay from window name
    @classmethod
    def stopDisplay(cls, windowName: str, timeout: Union[float, None] = None):
        cls.displays[windowName].stop()
        if cv2.getWindowProperty(windowName, 0) >= 0:
            cv2.destroyWindow(windowName)
        cls.displays[windowName].join(timeout=timeout)

    # stops all displays
    @classmethod
    def stopDisplays(cls, timeout: Union[float, None] = None):
        for source, thread in cls.displays.items():
            cls.stopDisplay(source, timeout)
