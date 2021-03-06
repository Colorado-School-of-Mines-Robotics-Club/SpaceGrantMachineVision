# Built in python libs
import sys
import time

# Additional libs
import cv2
import numpy as np
from openVO import StereoCamera

# Custom imports
from source.logger import Logger, logArguments, logSystemInfo, logConfiguration
from source.cameras import fetchCameraImages, initCameras, closeCameras, DisplayManager, loadCalibrationFiles,\
    PTframes, frameStaticBuilder
from source.visualOdometry import makeOdometer, updateOdometer
from source.features import computeMatchingPoints, getPointsFromKeypoints, getAvgTranslationXY
from source.objectDetection import objectDetection, experimental
from source.simulation import Map, Robot
from source.utilities import getAvgTimeArr, getArgDict, getArgFlags, handleRecordFlag, handleClearLogFlag,\
    handleVideoFlag, handleRecordFlagClose, handleThreadedDisplayFlag, Config, exceptions, jit_compile_all
from source.concurrency import PayloadManager
from source.hardware import PThardwareCommand, createHardwareManager, KinematicHardwareInterface, RobotData

from source.autonomous import autonomous
from source.remoteControl import remoteControl

# denotes program entered in this file, the main thread
if __name__ == "__main__":
    # get dictionary with cli args
    argDict = getArgDict()
    # sets global flags from boolean arguments
    HEADLESS, CLEAR_LOG, RECORD, THREADED_DISPLAY, REMOTE_CONTROL = getArgFlags(argDict)

    # load the configuration file
    Config.init()
    runParameters = Config.getRunParameters()
    loggingOptions = Config.getLoggingOptions()
    iterationConstants = Config.getIterationConstantsDict()
    cameraParams = Config.getCamerasDict()
    orbParams = Config.getOrbParamsDict()
    featureParams = Config.getFeatureParamsDict()
    objectDetectionParams = Config.getObjectDetectionDict()
    sgbmPs = Config.getSGBMParamsDict()
    odometerParams = Config.getOdometerParamsDict()

    wlsParams = Config.getWLSParamsDict()
    hardwarePorts = Config.getElectronicPortsDict()

    LOG_ITERATION_INFO = loggingOptions['logIterationStarts']

    # Merge config file run parameters with runtime args
    HEADLESS = HEADLESS or runParameters['headless']
    CLEAR_LOG = CLEAR_LOG or runParameters['clearlog']
    RECORD = RECORD or runParameters['record']
    THREADED_DISPLAY = THREADED_DISPLAY or runParameters['threadeddisplay']
    VIDEO_PATH = argDict['video']
    if not runParameters['video'] == '':
        VIDEO_PATH = runParameters['video']
    if runParameters['remoteControl']:
        REMOTE_CONTROL = True
    CAMERAS_PATH = argDict['cameras']

    # clears log file if the CLEAR_LOG is present
    handleClearLogFlag(CLEAR_LOG)
    # begin logging and other startup methods for primary control flow
    Logger.init("log.log")  # Starts the logger and sets the logger to log to the specified file.
    # ensure logger init is done before logging anything
    time.sleep(1)
    # log system info
    if loggingOptions['logSystemInfo']:
        logSystemInfo(Logger)

    if loggingOptions['logParameters']:
        # log all arguments
        logArguments(Logger, argDict)
        # log all configuration details
        logConfiguration(Logger)
    
    if REMOTE_CONTROL:
        remoteControl(60.0)
        sys.exit(0)

    jit_compile_all(verbose=True)

    # Global constants for any hyper parameters for the code or physical constants
    # Define any global constants
    # defines the amount of skipped/incomplete iterations before the loop is restarted
    errorTolerance = iterationConstants['errorTolerance']
    iterationsToAverage = iterationConstants['iterationsToAverage']  # use n+1 to calculate true number averaged

    # defining opencv objects
    # orb feature detector object
    orb = cv2.ORB_create(nfeatures=orbParams['nfeatures'], scaleFactor=orbParams['scaleFactor'],
                         nlevels=orbParams['nlevels'], edgeThreshold=orbParams['edgeThreshold'],
                         firstLevel=orbParams['firstLevel'], patchSize=orbParams['patchSize'])
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # matcher object

    # inits the DisplayManager
    DisplayManager.init()

    leftCam, rightCam = handleVideoFlag(VIDEO_PATH, cameraParams["useCapDShow"], cameraParams['leftPort'],
                                        cameraParams['rightPort'])
    VIDEO = False if isinstance(leftCam, int) else True

    # Initialize openVO.StereoCamera for use in ThreadedCapture
    leftK, rightK, leftDistC, rightDistC, rectParams = loadCalibrationFiles(CAMERAS_PATH)

    # Need to find image size without ThreadedCapture because we need it to init StereoCamera,
    # which is needed to init ThreadedCapture. Hard coding for now because I'm not sure if we want
    # to read a frame manually or just add a config file
    frameSize = Config.getFrameSize()
    # stereo = StereoCamera(leftK, leftDistC, rightK, rightDistC, rectParams, sgbmPs, frameSize)

    # try:
    #     initCameras(leftCam, rightCam, stereo, setExposure=cameraParams['setExposure'])
    # except exceptions.CameraReadError:
    #     Logger.log("Could not open one or more of the cameras")
    #     time.sleep(1)
    #     sys.exit(1)

    leftWriter, rightWriter = handleRecordFlag(RECORD, leftCam, rightCam, fetchCameraImages)

    handleThreadedDisplayFlag(THREADED_DISPLAY, HEADLESS)

    # define the map
    worldMap = Map(D=15.24)
    robotData = RobotData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    robot = Robot()
    interface = KinematicHardwareInterface()

    # # TESTING FOR PAYLOAD MANAGER
    # stereo, leftCapture, rightCapture = frameStaticBuilder((leftCam, rightCam, leftK, rightK, leftDistC, rightDistC, rectParams, sgbmPs, frameSize))
    # data = PTframes((None, stereo, leftCapture, rightCapture, 5.0))

    # multiprocessing, defines payloads to be run in parallel
    payloads = list()
    payloads.append(("cameras", PTframes, (.005, not HEADLESS, THREADED_DISPLAY), frameStaticBuilder,
                     (leftCam, rightCam, leftK, rightK, leftDistC, rightDistC, rectParams, sgbmPs, frameSize), None))
    cameraArgs = (leftK, leftDistC, rightK, rightDistC, rectParams, sgbmPs, frameSize)
    payloads.append(("updateOdometer", updateOdometer, (not HEADLESS, THREADED_DISPLAY), makeOdometer,
                     (cameraArgs, odometerParams), None))
    payloads.append(("hardware", PThardwareCommand, (False,), createHardwareManager, (), None))
    # payloads.append(("clustering", experimental.runClustering, (not HEADLESS, THREADED_DISPLAY), None, None, None))
    PayloadManager.initStart(payloads)

    # being primary loop
    Logger.log("Program starting...")
    while True:
        try:
            Logger.log("Starting loop...")
            autonomous(HEADLESS, LOG_ITERATION_INFO, THREADED_DISPLAY, RECORD, VIDEO, errorTolerance,
                       iterationsToAverage, leftCam, rightCam, leftWriter, rightWriter, orb, objectDetectionParams,
                       worldMap, interface, robotData)
            Logger.log("Shutdown loop...")
            # sleep and then check for keyboardInterrupt will fully kill program
            time.sleep(1)
            keyPressed = cv2.waitKey(1) & 0xFF
            if keyPressed == 27:
                Logger.log("Starting Program shutdown...")
                break
        except exceptions.CameraReadError:
            sys.exit(1)
        except KeyboardInterrupt:
            Logger.log("Keyboard Interrupt handled in main")
            # print("Keyboard Interrupt handled in main")
            break

    Logger.log("    Closing processes through PayloadManager")
    PayloadManager.closeAll(timeout=0.5)
    Logger.log("    Closing cameras...")
    # closeCameras()
    Logger.log("    Closing video writers...")
    handleRecordFlagClose(leftWriter, rightWriter)
    Logger.log("    Closing displays through DisplayManager...")
    Logger.log("        & Closing main process displays...")
    DisplayManager.stopDisplays(timeout=0.5)
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    Logger.log("    Shutting down logger...")
    Logger.shutdown()  # Shuts down the logging system and prints a closing message to the file
    print("Exiting...")
    sys.exit(0)
