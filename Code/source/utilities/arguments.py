# Built in python libs
import sys
import os
from argparse import ArgumentParser, Namespace
from typing import Dict, Tuple

# Additional libs
import cv2
import numpy as np
import platform

# Custom imports
from source.logger.Logger import Logger
from source.cameras import fetchCameraImages
from .Config import Config


def getArguments() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument("-H", "--headless", help="Do not show debug windows", action='store_true', required=False)
    parser.add_argument("-TD", "--threadeddisplay", help="Use threads to speed up displays in headed mode",
                        action="store_true", required=False)
    parser.add_argument("-R", "--record", help="Record the camera inputs to videos", action='store_true',
                        required=False)
    parser.add_argument("-CL", "--clearlog", help="Clears the log on running the program", action='store_true',
                        required=False)
    parser.add_argument("-V", "--video", help="Set a video folder which contains a left and right camera feed",
                        nargs='?', const='Data/Cameras/DefaultVideo/')
    parser.add_argument("-C", "--config", help="Specify a different config file to use",
                        nargs='?', const='config.json', required=False)
    parser.add_argument("-RC", "--remote", help="Run the robot using a remote control system.", action="store_true",
                        required=False)
    args = parser.parse_args()
    return args


def getArgDict() -> Dict:
    # gets the arguments for the program
    args = getArguments()
    argDict = dict()
    argDict['headless'] = True if args.headless else False
    argDict['threadeddisplay'] = True if args.threadeddisplay else False
    uname = platform.uname()
    if uname.system != "Windows" or "Windows" not in uname.system:
        argDict['threadeddisplay'] = False
    argDict['record'] = True if args.record else False
    argDict['clearlog'] = True if args.clearlog else False
    # finds the video directory
    if args.video is not None:
        counter = 0
        while not os.path.isdir(args.video):
            args.video = "../" + args.video
            counter += 1
            if counter > 3:
                raise Exception("Video Argument: Could not find specified folder")
    argDict['video'] = args.video
    if args.config is None:
        argDict['config'] = 'config.json'
    else:
        if not os.path.isfile(args.config):
            raise Exception("Config Argument: Could not find specified config file")
        argDict['config'] = args.config
    argDict['remote'] = args.remote
    return argDict


def getArgFlags(argDict: Dict) -> Tuple[bool, bool, bool, bool, bool]:
    # HEADLESS, CLEAR_LOG, RECORD, THREADED_DISPLAY
    return argDict['headless'], argDict['clearlog'], argDict['record'], argDict['threadeddisplay'], argDict['remote']


# make video writers for record flag
def handleRecordFlag(RECORD: bool, leftCam: int, rightCam: int) -> Tuple[cv2.VideoWriter, cv2.VideoWriter]:
    # initiate writers
    leftWriter = None
    rightWriter = None
    if RECORD:
        Config.init()
        videoPath = Config.getFilepathsDict()['videoPath']
        while not os.path.isdir(videoPath):
            videoPath = "../" + videoPath
        leftImage, rightImage = fetchCameraImages(leftCam, rightCam)
        height, width, _ = leftImage.shape
        fourcc = cv2.VideoWriter_fourcc('W', 'M', 'V', '2')
        fps = 16.0
        leftWriter = cv2.VideoWriter(f"{videoPath}leftOutput.wmv", fourcc=fourcc, fps=fps, frameSize=(width, height))
        rightWriter = cv2.VideoWriter(f"{videoPath}rightOutput.wmv", fourcc=fourcc, fps=fps, frameSize=(width, height))
    return leftWriter, rightWriter


def handleRecordFlagClose(leftWriter: cv2.VideoWriter, rightWriter: cv2.VideoWriter):
    if leftWriter is not None:
        leftWriter.release()
    if rightWriter is not None:
        rightWriter.release()


# wipes the log ahead of the logger being restarted
def handleClearLogFlag(CLEAR_LOG: bool, logFile="log.log"):
    if CLEAR_LOG:
        try:
            with open(logFile, 'r+') as f:
                f.truncate(0)
                f.seek(0)
        except FileNotFoundError:
            pass


def handleVideoFlag(video: str, use_cap_dshow: bool, leftPort: int, rightPort: int) -> Tuple[str, str]:
    # loading data for cameras and starting the camera process
    if video is None:
        leftCam = leftPort
        rightCam = rightPort
        if use_cap_dshow:  # cv2.CAP_DSHOW changes internal api stuff for opencv
            leftCam += cv2.CAP_DSHOW
            rightCam += cv2.CAP_DSHOW
    else:
        leftCam = f"{video}stereo_left.avi"
        rightCam = f"{video}stereo_right.avi"
    return leftCam, rightCam


def handleThreadedDisplayFlag(THREADED_DISPLAY: bool, HEADLESS: bool):
    # if the displays are in threaded mode then we need a new screen to capture the keyboard
    if not HEADLESS and THREADED_DISPLAY:
        input_image = np.zeros((300, 300))
        cv2.imshow("Input Screen", input_image)
