# Built in python libs
import sys
import time

# Additional libs
import cv2

# Custom imports
from source.logger import Logger, logArguments, logSystemInfo, logConfiguration
from source.cameras import fetchAndShowCameras, initCameras, closeCameras, DisplayManager, CaptureManager
from source.visualOdometry import PTcomputeDisparity, makeStereoObjects
from source.features import computeMatchingPoints, getPointsFromKeypoints, getAvgTranslationXY
from source.objectDetection import objectDetection
from source.simulation import Map, Robot
from source.utilities import getAvgTimeArr, getArgDict, getArgFlags, handleRecordFlag, handleClearLogFlag,\
    handleVideoFlag, handleRecordFlagClose, handleThreadedDisplayFlag, Config, exceptions, jit_compile_all
from source.concurrency import PayloadManager
from source.hardware import PThardwareCommand, createHardwareManager

from source.autonomous import autonomous
from source.remoteControl import remoteControl


# denotes program entered in this file, the main thread
if __name__ == "__main__":
    # get dictionary with cli args
    argDict = getArgDict()
    # sets global flags from boolean arguments
    HEADLESS, CLEAR_LOG, RECORD, THREADED_DISPLAY, REMOTE_CONTROL = getArgFlags(argDict)

    if REMOTE_CONTROL:
        remoteControl(60.0)
        sys.exit(0)

    jit_compile_all(verbose=True)
    # load the configuration file
    Config.init(argDict['config'])
    runParameters = Config.getRunParameters()
    loggingOptions = Config.getLoggingOptions()
    iterationConstants = Config.getIterationConstantsDict()
    cameraParams = Config.getCamerasDict()
    orbParams = Config.getOrbParamsDict()
    featureParams = Config.getFeatureParamsDict()
    objectDetectionParams = Config.getObjectDetectionDict()
    sbgmPs = Config.getSBGMParamsDict()
    wlsParams = Config.getWLSParamsDict()
    hardwarePorts = Config.getHardwarePortsDict()

    LOG_ITERATION_INFO = loggingOptions['logIterationStarts']

    # Merge config file run parameters with runtime args
    HEADLESS = HEADLESS or runParameters['headless']
    CLEAR_LOG = CLEAR_LOG or runParameters['clearlog']
    RECORD = RECORD or runParameters['record']
    THREADED_DISPLAY = THREADED_DISPLAY or runParameters['threadeddisplay']
    VIDEO_PATH = argDict['video']
    if not runParameters['video'] == '':
        VIDEO_PATH = runParameters['video']

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

    try:
        initCameras(leftCam, rightCam, setExposure=cameraParams['setExposure'])
    except exceptions.CameraReadError:
        Logger.log("Could not open one or more of the cameras")
        time.sleep(1)
        sys.exit(1)

    leftWriter, rightWriter = handleRecordFlag(RECORD, leftCam, rightCam)

    handleThreadedDisplayFlag(THREADED_DISPLAY, HEADLESS)

    # define the map
    Map = Map()
    Robot = Robot()

    # multiprocessing, defines payloads to be run in parallel
    payloads = list()
    payloads.append(("disparity", PTcomputeDisparity, (not HEADLESS, THREADED_DISPLAY), makeStereoObjects, (), None))
    payloads.append(("hardware", PThardwareCommand, (), createHardwareManager, (), None))
    PayloadManager.initStart(payloads)

    # being primary loop
    Logger.log("Program starting...")
    while True:
        try:
            Logger.log("Starting loop...")
            autonomous(HEADLESS, LOG_ITERATION_INFO, THREADED_DISPLAY, RECORD, errorTolerance, iterationsToAverage,
                       leftCam, rightCam, leftWriter, rightWriter, orb, matcher, featureParams, objectDetectionParams)
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
    closeCameras()
    Logger.log("    Closing video writers...")
    handleRecordFlagClose(leftWriter, rightWriter)
    Logger.log("    Closing displays through DisplayManager...")
    Logger.log("        & Closing main process displays...")
    DisplayManager.stopDisplays(timeout=0.5)
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    Logger.log("    Shutting down logger...")
    Logger.shutdown()  # Shuts down the logging system and prints a closing message to the file
    sys.exit(0)
