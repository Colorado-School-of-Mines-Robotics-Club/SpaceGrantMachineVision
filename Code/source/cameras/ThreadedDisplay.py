from threading import Thread
from collections import deque
import cv2
from cv2 import error as cv2Error
from typing import Union
import time
import numpy as np


class ThreadedDisplay:
    """
    Class that continuously shows a frame using a dedicated thread.
    """

    def __init__(self, windowName="Output", frame=None, fps=24.0):
        self.windowName = windowName
        self.stopped = False
        self.queue = deque(maxlen=2)
        if frame is not None:
            self.queue.append(frame)
        self.delay = 1.0 / fps
        self.thread = None

    def start(self) -> 'ThreadedDisplay':
        self.thread = Thread(target=self.show, args=(), name=f"{self.windowName} Display Thread")
        self.thread.setDaemon(True)
        self.thread.start()
        return self

    def show(self):
        while not self.stopped:
            try:
                cv2.imshow(self.windowName, self.queue.popleft())
                keyPressed = cv2.waitKey(1) & 0xFF
                if keyPressed == 27:
                    self.stopped = True
            except IndexError:
                pass
            time.sleep(self.delay)

    def updateFPS(self, fps: float):
        self.delay = 1.0 / fps

    def update(self, newFrame: np.ndarray):
        self.queue.append(newFrame)

    def join(self, timeout: Union[float, None] = None):
        self.thread.join(timeout=timeout)

    def stop(self):
        self.stopped = True

    def isAlive(self):
        return self.thread.is_alive()
