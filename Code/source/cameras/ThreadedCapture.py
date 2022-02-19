from threading import Thread
from collections import deque
import cv2
import os
import time
from source.logger.Logger import Logger
from source.utilities.Config import Config


class ThreadedCapture:
    """
        Class that continuously gets frames from a VideoCapture object
        with a dedicated thread.
    """
    def __init__(self, source, fps=None, delayOffset=1.0, K=None, distC=None, setExposure=False, autoExposure=1.0,
                 exposure=100.0, framesAutoFPS=5):
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
        if isinstance(source, str):
            if os.path.exists(source):
                self.video = True
                self.frameQ = deque()
            else:
                raise FileNotFoundError(f"Could not find file for source:{source}")
        # basic checking with asserts that all data is present
        if (K is not None) or (distC is not None):
            assert ((K is not None) and (distC is not None)), "If K or distC is defined, then both must be defined"
        # define flow controllers
        self.stopped = False
        self.success = True
        # define source, camera intrinsic matrix, distortion coefficients, and exposure settings
        self.source = source
        self.frame = None
        self.K = K
        # results from cv2.getOptimalNewCameraMatrix
        self.newK, self.roi, self.x, self.y, self.w, self.h = None, None, None, None, None, None
        self.distC = distC
        self.setExposure = setExposure
        self.autoExposure = autoExposure
        self.exposure = exposure
        # create the cv2 video capture to acquire either the recorded video or webcam
        try:
            self.capture = cv2.VideoCapture(source)
        except Exception:
            raise Exception(f'Error defining cv2.videoCapture object for source: {self.source}')
        if not self.capture.isOpened():
            raise Exception(f"Could not open video source: {self.source}")

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
        # create undistortion K matrix
        try:
            success, frame = self.capture.read()
            if (self.K is not None) and (self.distC is not None):
                h, w = frame.shape[:2]
                self.newK, self.roi = cv2.getOptimalNewCameraMatrix(self.K, self.distC, (w, h), 1, (w, h))
                self.x, self.y, self.w, self.h = self.roi
                if self.LOG_CAMERA_RECTIFICATION:
                    printK = str(self.K).replace('\n', '')
                    printNewK = str(self.newK).replace('\n', '')
                    printDistC = str(self.distC).replace('\n', '')
                    Logger.log(f"   K: {printK}")
                    Logger.log(f"   New K: {printNewK}")
                    Logger.log(f"   Distortion Coefficients: {printDistC}")
                    Logger.log(f"   Width: {self.w}, Height: {self.h}")
        except Exception as e:
            raise Exception(f'Error computing new K matrix for video source: {self.source} -> {e}')
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
        if self.newK is not None:
            frame = cv2.undistort(frame, self.K, self.distC, None, self.newK)
            frame = frame[self.y:self.y+self.h, self.x:self.x+self.w]
        if self.frameQ is not None:
            self.frameQ.append(frame)
        else:
            self.frame = frame

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

    def addFrame(self, frame):
        if self.frameQ is not None:
            self.frameQ.append(frame)

    # returns the current frame
    def getFrame(self):
        if self.frameQ is not None:
            try:
                return self.frameQ.popleft()
            except IndexError:
                return None
        return self.frame

    # TODO: figure out to how strongly type return value of self for class
    # starts the capture thread
    def start(self):
        thread = Thread(target=self.readFrames, args=())
        thread.setDaemon(True)
        thread.start()
        return self

    # stops the capture
    def stop(self):
        self.stopped = True
