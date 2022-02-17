from typing import List, Dict, Union
import numpy as np

from .ThreadedCapture import ThreadedCapture
from source.logger.Logger import Logger


def createCaptureSourceData(source: Union[str, int], fps=None, delayOffset=1.0, K=None, distC=None, setExposure=False,
                            autoExposure=1.0, exposure=100.0, framesAutoFPS=5) -> List:
    return [source, fps, delayOffset, K, distC, setExposure, autoExposure, exposure, framesAutoFPS, log]


class CaptureManager:
    """
        Class that managers threadedCapture objects and allows access to
        frames acquired by those objects.
    """
    # generate a dict indexed by source to aggregate the threads
    sources = dict()
    # sources is a list of lists formed where each entry should be created by createSourceData
    @classmethod
    def init(cls, sources: List):
        # initialize and start all threads
        for src in sources:
            cls.sources[src[0]] = ThreadedCapture(src[0], src[1], src[2], src[3], src[4], src[5], src[6], src[7],
                                                  src[8]).start()

    # gets the frame from a specific source
    @classmethod
    def getFrame(cls, source: Union[str, int]) -> np.ndarray:
        return cls.sources[source].getFrame()

    # gets all frames from all sources
    @classmethod
    def getFrames(cls) -> Dict:
        frames = dict()
        for source, thread in cls.sources.items():
            frames[source] = thread.getFrame()
        return frames

    @classmethod
    def updateFPS(cls, source: Union[str, int], fps: float, delayOffset=1.0):
        cls.sources[source].updateFPS(fps, delayOffset)

    @classmethod
    def updateAllFPS(cls, fps: float, delayOffset=1.0):
        for source, thread in cls.sources.items():
            thread.updateFPS(fps, delayOffset)

    # stops a threadedCapture for source
    @classmethod
    def stopSource(cls, source: Union[str, int]):
        cls.sources[source].stop()

    # stops all sources
    @classmethod
    def stopSources(cls):
        for source, thread in cls.sources.items():
            thread.stop()
