from threading import Thread
from collections import deque
import cv2
import os
import time
import numpy as np

from typing import Union, Tuple
from enum import Enum
from openVO import StereoCamera

try:
    from source.logger.Logger import Logger
    from source.utilities.Config import Config
    from source.utilities.exceptions import CameraReadError
except ModuleNotFoundError:
    from Code.source.logger.Logger import Logger
    from Code.source.utilities.Config import Config
    from Code.source.utilities.exceptions import CameraReadError

class ThreadedCapture:
    """
        Class that continuously gets frames from a VideoCapture object
        with a dedicated thread.
    """

    class CameraType(Enum):
        LEFT = 0
        RIGHT = 1

    def __init__(self, stereo: StereoCamera, source: Union[str, int], type: CameraType,
                 fps=None, delayOffset=1.0, setExposure=False,
                 autoExposure=1.0, exposure=100.0, framesAutoFPS=5):
        Config.init()
        self.LOG_SETUP = Config.getLoggingOptions()['logThreadedCaptureSetup']
        self.LOG_FRAME_INFO = Config.getLoggingOptions()['logFrameInfo']
        self.LOG_VIDEO_INPUT_INFO = Config.getLoggingOptions()['logVideoInputInfo']
        self.LOG_CAMERA_RECTIFICATION = Config.getLoggingOptions()['logCameraRectification']
        # define delay from fps
        # if fps does not exist then define it automatically at the end of init
        if fps is not None:
            self.delay = 1.0 / (fps - delayOffset)
        # check if the source is a video file
        self.video = False
        self.frameQ = None
        self.cframeQ = None
        if isinstance(source, str):
            if os.path.exists(source):
                self.video = True
                self.frameQ = deque()
                self.cframeQ = deque()
            else:
                raise FileNotFoundError(f"Could not find file for source:{source}")
        # define flow controllers
        self.stopped = False
        self.success = True
        # define stereo configuration, video source, camera type, and exposure settings
        self.stereo = stereo
        self.source = source
        self.type = type
        self.frame = None
        self.cropped = None
        self.setExposure = setExposure
        self.autoExposure = autoExposure
        self.exposure = exposure
        # create the cv2 video capture to acquire either the recorded video or webcam
        try:
            self.capture = cv2.VideoCapture(source)
        except Exception:
            raise Exception(f'Error defining cv2.videoCapture object for source: {self.source}')
        if not self.capture.isOpened():
            raise CameraReadError(f"Could not open video source: {self.source}")

        try:
            startTime = time.time()
            while True:
                got_frame, temp_frame = self.capture.read()
                if got_frame:
                    if self.LOG_VIDEO_INPUT_INFO:
                        # Print video source info
                        Logger.log(f'Capture <{source}> info:')
                        Logger.log(f'   Resolution: {temp_frame.shape[1]} x {temp_frame.shape[0]}')
                        imageFormat = temp_frame.shape[2]
                        if imageFormat == 1:
                            'Grayscale'
                        elif imageFormat == 3:
                            imageFormat = 'RGB'
                        elif imageFormat == 4:
                            imageFormat = 'RGBA'
                        else:
                            imageFormat = f'{imageFormat} channels'
                        Logger.log(f'   Format: {imageFormat}')
                    break
                if time.time() - startTime > 5:
                    raise Exception("Timed out in camera read")
                time.sleep(0.1)
        except Exception as e:
            raise Exception(f"Error reading from video source: {self.source} -> {e}")
        # set exposures if option set
        try:
            if self.setExposure:
                self.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, autoExposure)
                self.capture.set(cv2.CAP_PROP_EXPOSURE, exposure)
        except Exception as e:
            raise Exception(f"Error settings exposure for video source: {self.source} -> {e}")
        # automatic fps calibration
        if fps is None:
            start = time.perf_counter()
            for i in range(0, framesAutoFPS):
                _, _ = self.capture.read()
            self.delay = 1.0 / ((framesAutoFPS / (time.perf_counter() - start)) - delayOffset)
        if self.LOG_SETUP:
            Logger.log(f"  Completed setup of video source: {self.source} @ {time.perf_counter()}")

    # updates the fps of the camera (and optionally the delayOffset)
    def updateFPS(self, fps: float, delayOffset=1.0):
        self.delay = 1 / (fps - delayOffset)

    # reads the most recent image from the camera and saves it to self.frame
    def readCapture(self):
        self.success, frame = self.capture.read()
        if not self.success:
            self.stopped = True
            return

        if (self.type == ThreadedCapture.CameraType.LEFT):
            frame = self.stereo.undistort_rectify_left(frame)
            cropped = self.stereo.crop_to_valid_region_left(frame)
        else:
            frame = self.stereo.undistort_rectify_right(frame)
            cropped = self.stereo.crop_to_valid_region_right(frame)

        if self.frameQ is not None:
            self.frameQ.append(frame)
            self.cframeQ.append(cropped)
        else:
            self.frame = frame
            self.cropped = cropped

    def readFrames(self):
        while not self.stopped:
            if not self.success:
                self.stop()
            else:
                self.readCapture()
                if self.LOG_FRAME_INFO:
                    Logger.log(f"  {self.source}: Queued frame @ {time.perf_counter()}") if self.frameQ else Logger.log(
                        f"  {self.source}: Updated frame @ {time.perf_counter()}")
            time.sleep(self.delay)
        self.capture.release()

    def addFrame(self, frame, cropped):
        if self.frameQ is not None:
            self.frameQ.append(frame)
            self.cframeQ.append(cropped)

    # returns the current frame
    def getFrame(self) -> Union[Tuple[np.ndarray, np.ndarray], Tuple[None, None]]:
        if self.frameQ is not None:
            try:
                return self.frameQ.popleft(), self.cframeQ.popleft()
            except IndexError:
                return None, None
        return self.frame, self.cropped

    # starts the capture thread
    def start(self) -> 'ThreadedCapture':
        thread = Thread(target=self.readFrames, args=(), name=f"{self.capture} Capture Thread", daemon=True)
        thread.start()
        return self

    # stops the capture
    def stop(self):
        self.stopped = True
