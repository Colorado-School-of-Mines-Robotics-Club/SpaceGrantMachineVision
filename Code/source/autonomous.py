# Built in python libs
import sys
import time

# Additional libs
import cv2

# Custom imports
from .logger import Logger, logArguments, logSystemInfo, logConfiguration
from .cameras import fetchAndShowCameras, initCameras, closeCameras, DisplayManager, CaptureManager
from .visualOdometry import PTcomputeDisparity, makeStereoObjects
from .features import computeMatchingPoints, getPointsFromKeypoints, getAvgTranslationXY
from .objectDetection import objectDetection
from .simulation import Map, Robot
from .utilities import getAvgTimeArr, getArgDict, getArgFlags, handleRecordFlag, handleClearLogFlag,\
    handleVideoFlag, handleRecordFlagClose, handleThreadedDisplayFlag, Config, exceptions, jit_compile_all
from .concurrency import PayloadManager


def autonomous(HEADLESS, LOG_ITERATION_INFO, THREADED_DISPLAY, RECORD, errorTolerance, iterationsToAverage, leftCam,
               rightCam, leftWriter, rightWriter, orb, matcher, featureParams, objectDetectionParams):
    numTotalIterations, consecutiveErrors, iterationCounter, iterationTime = 0, 0, 0, 0
    iterationTimes, cameraFTs, featureFTs, objectDectFTs, disparityFTs = list(), list(), list(), list(), list()
    leftImage, rightImage, grayLeftImage, grayRightImage = None, None, None, None
    leftPts, rightPts, leftKp, leftDesc, rightKp, rightDesc = None, None, None, None, None, None
    leftMatches, rightMatches = None, None
    xTranslation, yTranslation = None, None
    objectBoundingBoxes = None
    disparityMap = None
    while True:
        iterationStartTime = time.perf_counter()
        if LOG_ITERATION_INFO:
            Logger.log(f"#{numTotalIterations}: Started @ {iterationStartTime}")
        try:
            # need to save previous images (if they exist) for visual odometry
            prevLeftImage, prevRightImage = leftImage, rightImage
            prevGrayLeftImage, prevGrayRightImage = grayLeftImage, grayRightImage

            # save previous feature information
            prevLeftPts, prevRightPts = leftPts, rightPts
            prevLeftKp, prevRightKp = leftKp, rightKp
            prevLeftDesc, prevRightDesc = leftDesc, rightDesc
            prevLeftMatches, prevRightMatches = leftMatches, rightMatches
            prevObjectBoundingBoxes = objectBoundingBoxes

            # save previous frame visual odometry information
            prevXTranslation, prevYTranslation = xTranslation, yTranslation
            prevDisparityMap = disparityMap

            cameraStartTime = time.perf_counter()
            # Satisfies that read images stage of control flow
            leftImage, rightImage, grayLeftImage, grayRightImage = fetchAndShowCameras(leftCam, rightCam,
                                                                                       show=not HEADLESS,
                                                                                       threadedDisplay=THREADED_DISPLAY)
            PayloadManager.addInputs('disparity', [grayLeftImage, grayRightImage])
            cameraFTs.append(time.perf_counter() - cameraStartTime)

            featureStartTime = time.perf_counter()
            # feature points for left and right images
            # the point at index [0], [1], [2], etc. in both is the same real life feature
            # COMPUTES MATCHING FEATURES ACROSS BOTH CURRENT IMAGES
            prevLeftPts, leftPts, prevLeftKp, prevLeftDesc, leftKp, leftDesc, leftMatches = \
                computeMatchingPoints(prevGrayLeftImage, grayLeftImage, orb, matcher, prevLeftKp, prevLeftDesc,
                                      ratio=featureParams['startingRatio'],
                                      featureRatio=featureParams["featureRatio"], stepSize=featureParams["stepSize"],
                                      timeout=featureParams["timeout"], show=not HEADLESS,
                                      threadedDisplay=THREADED_DISPLAY, windowName="LeftCaptures Matched Features")
            prevRightPts, rightPts, prevRightKp, prevRightDesc, rightKp, rightDesc, rightMatches = \
                computeMatchingPoints(prevGrayRightImage, grayRightImage, orb, matcher, prevRightKp, prevRightDesc,
                                      ratio=featureParams['startingRatio'],
                                      featureRatio=featureParams["featureRatio"], stepSize=featureParams["stepSize"],
                                      timeout=featureParams["timeout"], show=not HEADLESS,
                                      threadedDisplay=THREADED_DISPLAY, windowName="Right Matched Features")
            xTranslation, yTranslation = getAvgTranslationXY(leftMatches, prevLeftKp, leftKp, rightMatches, prevRightKp,
                                                             rightKp)
            featureFTs.append(time.perf_counter() - featureStartTime)

            objectDectStartTime = time.perf_counter()
            # acquires the bounding box cordinates for areas of the image where there are dense features
            objectBoundingBoxes = objectDetection(leftImage, getPointsFromKeypoints(leftKp),
                                                  binSize=objectDetectionParams["binSize"],
                                                  featuresPerPixel=objectDetectionParams["featuresPerPixel"],
                                                  simplifyFinalOutput=True, show=not HEADLESS,
                                                  threadedDisplay=THREADED_DISPLAY)
            objectDectFTs.append(time.perf_counter() - objectDectStartTime)

            disparityStartTime = time.perf_counter()
            disparityMap = PayloadManager.getOutput('disparity')
            disparityFTs.append(time.perf_counter() - disparityStartTime)


            # all additional functionality should be present within the === comments
            # additional data that needs to be stored for each iteration should be handled above
            # ===========================================================================================================
            # TODO
            # Fill in remainder of functionality

            # # ===========================================================================================================
            # # redraws the map
            # mapDisplay = Map.draw()
            # mapDisplay = Robot.draw(mapDisplay)
            # if not HEADLESS:
            #     if THREADED_DISPLAY:
            #         DisplayManager.show("Current Map", mapDisplay)
            #     else:
            #         cv2.imshow("Current Map", mapDisplay)
            # handles saving the video feed
            if RECORD:
                leftWriter.write(leftImage)
                rightWriter.write(rightImage)
            # Resets the consecutive error count if a full iteration is completed
            consecutiveErrors = 0
            # cv2.waitKey is needed for opencv to properly display images (think of it like a timer or interrupt)
            if not HEADLESS:
                mainLoopKeyPressed = cv2.waitKey(1) & 0xFF
                if mainLoopKeyPressed == 27:
                    raise exceptions.CustomKeyboardInterrupt("ESC")  # Quit on ESC
            iterationTime = time.perf_counter() - iterationStartTime
        except exceptions.CustomKeyboardInterrupt as e:  # Kills the loop if a keyboardInterrupt occurs
            Logger.log(f"User killed loop with: {e.getKey()}")
            break
        except KeyboardInterrupt as e:
            raise e
        except exceptions.CameraReadError:
            break
        except Exception as e:
            # Possibly instead of restarting, we might want to look into
            Logger.log(
                f"{str(e)} -> Occured in primary operation loop of program. " +
                f"Failed iterations in a row: {consecutiveErrors}")
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
