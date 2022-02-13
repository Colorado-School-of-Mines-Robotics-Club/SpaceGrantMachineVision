# Built in python libs
import os
import sys
import time

# Additional libs
import numpy as np
import cv2

# Custom imports
from source.logger import Logger, logArguments, logSystemInfo, logConfiguration
from source.cameras import fetchAndShowCameras, initCameras, closeCameras, DisplayManager, CaptureManager
from source.visualOdometry import computeDisparity
from source.features import computeMatchingPoints, getPointsFromKeypoints
from source.objectDetection import objectDetection
from source.utilities import getAvgTimeArr, getArgDict, getArgFlags, handleRecordFlag, handleClearLogFlag,\
    handleVideoFlag, handleRecordFlagClose, handleThreadedDisplayFlag, Config, exceptions


# Primary function where our main control flow will happen
# Contains a while true loop for continous iteration
def main():
    numTotalIterations, consecutiveErrors, iterationCounter, iterationTime = 0, 0, 0, 0
    iterationTimes, cameraFTs, featureFTs, objectDectFTs, disparityFTs = list(), list(), list(), list(), list()
    leftImage, rightImage, grayLeftImage, grayRightImage = None, None, None, None
    leftPts, rightPts, leftKp, leftDesc, rightKp, rightDesc = None, None, None, None, None, None
    featureDenseBoundingBoxes = None
    disparityMap = None
    while True:
        iterationStartTime = time.perf_counter()
        Logger.log(f"#{numTotalIterations}: Started @ {iterationStartTime}")
        try:
            # need to save previous images (if they exist) for visual odometry
            prevLeftImage, prevRightImage = leftImage, rightImage
            prevGrayLeftImage, prevGrayRightImage = grayLeftImage, grayRightImage

            # save previous feature information
            prevLeftPts, prevRightPts = leftPts, rightPts
            prevLeftKp, prevRightKp = leftKp, rightKp
            prevLeftDesc, prevRightDesc = leftDesc, rightDesc
            prevFeatureDenseBoundingBoxes = featureDenseBoundingBoxes

            # save previous frame visual odometry information
            prevDisparityMap = disparityMap

            cameraStartTime = time.perf_counter()
            # Satisfies that read images stage of control flow
            leftImage, rightImage, grayLeftImage, grayRightImage = fetchAndShowCameras(leftCam, rightCam,
                                                                                       show=not HEADLESS,
                                                                                       threadedDisplay=THREADED_DISPLAY)
            cameraFTs.append(time.perf_counter() - cameraStartTime)

            featureStartTime = time.perf_counter()
            # feature points for left and right images
            # the point at index [0], [1], [2], etc. in both is the same real life feature,
            leftPts, rightPts, leftKp, leftDesc, rightKp, rightDesc = \
                computeMatchingPoints(grayLeftImage, grayRightImage, orb, matcher, show=not HEADLESS,
                                      threadedDisplay=THREADED_DISPLAY)
            featureFTs.append(time.perf_counter() - featureStartTime)

            objectDectStartTime = time.perf_counter()
            # acquires the bounding box cordinates for areas of the image where there are dense features
            objectBoundingBoxes = objectDetection(leftImage, getPointsFromKeypoints(leftKp), binSize=30.0,
                                                  featuresPerPixel=0.03, show=not HEADLESS,
                                                  threadedDisplay=THREADED_DISPLAY)
            objectDectFTs.append(time.perf_counter() - objectDectStartTime)

            disparityStartTime = time.perf_counter()
            # this disparity map calculation should maybe get removed since we ??only?? care about the depth values
            disparityMap = computeDisparity(stereo, grayLeftImage, grayRightImage, show=not HEADLESS,
                                            threadedDisplay=THREADED_DISPLAY)
            disparityFTs.append(time.perf_counter() - disparityStartTime)

            # all additional functionality should be present within the === comments
            # additional data that needs to be stored for each iteration should be handled above
            # ===========================================================================================================
            # TODO
            # Fill in remainder of functionality

            # ===========================================================================================================
            # handles saving the video feed
            if RECORD:
                leftWriter.write(leftImage)
                rightWriter.write(rightImage)
            # Resets the consecutive error count if a full iteration is completed
            consecutiveErrors = 0
            # cv2.waitKey is needed for opencv to properly display images (think of it like a timer or interrupt)
            if not HEADLESS:
                keyPressed = cv2.waitKey(1) & 0xFF
                if keyPressed == 27:
                    raise exceptions.CustomKeyboardInterrupt("ESC")  # Quit on ESC
            iterationTime = time.perf_counter() - iterationStartTime
        except exceptions.CustomKeyboardInterrupt as e:  # Kills the loop if a keyboardInterrupt occurs
            Logger.log(f"User killed loop with: {e.getKey()}")
            break
        except KeyboardInterrupt as e:
            raise e
        except Exception as e:
            # Possibly instead of restarting, we might want to look into
            Logger.log(
                f"{str(e)} -> Occured in primary operation loop of program. " +
                "Failed iterations in a row: {consecutiveErrors}")
            consecutiveErrors += 1
            if consecutiveErrors > errorTolerance:
                Logger.log("RESTARTING PRIMARY CONTROL LOOP")
                break
        if iterationCounter < iterationsToAverage:
            if iterationCounter != 0:
                iterationTimes.append(iterationTime)
            iterationCounter += 1
        else:
            avgIterTime = getAvgTimeArr(iterationTimes, iterationCounter)
            iterNum = f"#{numTotalIterations + 1} Total Iterations: "
            iterTimeStr = f"Avg iteration: {avgIterTime} ms"
            cameraTimeStr = f" => Avg frame: {getAvgTimeArr(cameraFTs, iterationCounter)} ms"
            featureTimeStr = f", Avg features: {getAvgTimeArr(featureFTs, iterationCounter)} ms"
            objectDectTimeStr = f", Avg object detection: {getAvgTimeArr(objectDectFTs, iterationCounter)} ms"
            disparityTimeStr = f", Avg disparity map: {getAvgTimeArr(disparityFTs, iterationCounter)} ms"
            Logger.log(iterNum + iterTimeStr + cameraTimeStr + featureTimeStr + objectDectTimeStr + disparityTimeStr)
            iterationCounter = 0
            iterationTimes, cameraFTs, featureFTs, objectDectFTs, disparityFTs = list(), list(), list(), list(), list()
            CaptureManager.updateAllFPS(1000.0 / avgIterTime, delayOffset=0.0)
            if not HEADLESS:
                DisplayManager.updateAllFPS(1000.0 / avgIterTime)
        numTotalIterations += 1


# denotes program entered in this file, the main thread
if __name__ == "__main__":
    # load the configuration file
    Config.init()
    runParameters = Config.getRunParameters()
    iterationConstants = Config.getIterationConstantsDict()
    cameraPorts = Config.getCameraPortsDict()
    orbParams = Config.getOrbParamsDict()
    sgbmParams = Config.getSBGMParamsDict()
    hardwarePorts = Config.getHardwarePortsDict()

    # get dictionary with args
    argDict = getArgDict()
    # sets global flags from boolean arguments
    HEADLESS, CLEAR_LOG, RECORD, THREADED_DISPLAY = getArgFlags(argDict)

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
    logSystemInfo(Logger)
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
    # stereo object
    stereo = cv2.StereoSGBM_create(minDisparity=sgbmParams['minDisparity'], numDisparities=sgbmParams['numDisparities'],
                                   blockSize=sgbmParams['blockSize'], P1=sgbmParams['P1'], P2=sgbmParams['P2'],
                                   disp12MaxDiff=sgbmParams['disp12MaxDiff'], preFilterCap=sgbmParams['preFilterCap'],
                                   uniquenessRatio=sgbmParams['uniquenessRatio'],
                                   speckleWindowSize=sgbmParams['speckleWindowSize'],
                                   speckleRange=sgbmParams['speckleRange'])

    # inits the DisplayManager
    DisplayManager.init()

    leftCam, rightCam = handleVideoFlag(VIDEO_PATH, cameraPorts['use_cap_dshow'], cameraPorts['leftPort'], cameraPorts['rightPort'])

    initCameras(leftCam, rightCam, setExposure=cameraPorts['set_exposure'])

    leftWriter, rightWriter = handleRecordFlag(RECORD, leftCam, rightCam)

    if not HEADLESS:
        handleThreadedDisplayFlag(THREADED_DISPLAY)

    # being primary loop
    Logger.log("Program starting...")
    while True:
        try:
            Logger.log("Starting loop...")
            main()
            Logger.log("Shutdown loop...")
            # sleep and then check for keyboardInterupt will fully kill program
            time.sleep(1)
            keyPressed = cv2.waitKey(1) & 0xFF
            if keyPressed == 27:
                Logger.log("Starting Program shutdown...")
                break
        except KeyboardInterrupt:
            Logger.log("Keyboard Interrupt handled in main")
            print("Keyboard Interrupt handled in main")
            break

    Logger.log("    Closing cameras...")
    closeCameras()
    Logger.log("    Closing video writers...")
    handleRecordFlagClose(leftWriter, rightWriter)
    Logger.log("    Closing displays through DisplayManager...")
    DisplayManager.stopDisplays()
    Logger.log("    Closing main process displays...")
    if not HEADLESS and THREADED_DISPLAY:
        cv2.destroyWindow("Input Screen")
    Logger.log("    Shutting down logger...")
    Logger.shutdown()  # Shuts down the logging system and prints a closing message to the file
    sys.exit(0)
