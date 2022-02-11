from typing import List, Dict
import numpy as np

from .ThreadedCapture import ThreadedCapture
from Source.logger.Logger import Logger

def createCaptureSourceData(source: int, K=None, distC=None, setExposure=False, autoExposure=1.0, exposure=100.0,
                            logger=None) -> List:
    return [source, K, distC, setExposure, autoExposure, exposure, logger]

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
            cls.sources[src[0]] = ThreadedCapture(src[0], src[1], src[2], src[3], src[4], src[5], src[6]).start()

    # gets the frame from a specific source
    @classmethod
    def getFrame(cls, source: int) -> np.ndarray:
        return cls.sources[source].getFrame()

    # gets all frames from all sources
    @classmethod
    def getFrames(cls) -> Dict:
        frames = dict()
        for source, thread in cls.sources.items():
            frames[source] = thread.getFrame()
        return frames

    # stops a threadedCapture for source
    @classmethod
    def stopSource(cls, source: int):
        cls.sources[source].stop()

    # stops all sources
    @classmethod
    def stopSources(cls):
        for source, thread in cls.sources.items():
            thread.stop()
