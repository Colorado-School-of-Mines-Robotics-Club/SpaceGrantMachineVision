from threading import Thread
from collections import deque
import cv2
import time
import numpy as np

class ThreadedDisplay:
    """
    Class that continuously shows a frame using a dedicated thread.
    """

    def __init__(self, windowName="Output", frame=None, fps=24.0):
        self.windowName = windowName
        self.stopped = False
        self.queue = deque()
        if frame is not None:
            self.queue.append(frame)
        self.delay = 1.0 / fps

    # TODO: figure out to how strongly type return value of self for class
    def start(self):
        thread = Thread(target=self.show, args=())
        thread.setDaemon(True)
        thread.start()
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
        cv2.destroyWindow(self.windowName)

    def update(self, newFrame: np.ndarray):
        self.queue.append(newFrame)

    def stop(self):
        self.stopped = True