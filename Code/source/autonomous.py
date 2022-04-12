# Built in python libs
import sys
import time

# Additional libs
import cv2

# Custom imports
from .logger import Logger, logArguments, logSystemInfo, logConfiguration
from .cameras import fetchAndShowCameras, DisplayManager, CaptureManager
from .features import computeMatchingPoints, getPointsFromKeypoints
from .objectDetection import objectDetection, experimental
from .simulation import Map, Robot
from .utilities import getAvgTimeArr, getArgDict, getArgFlags, handleRecordFlag, handleClearLogFlag,\
    handleVideoFlag, handleRecordFlagClose, handleThreadedDisplayFlag, Config, exceptions, jit_compile_all
from .concurrency import PayloadManager
from .pathfinding import astar


def autonomous(HEADLESS, LOG_ITERATION_INFO, THREADED_DISPLAY, RECORD, errorTolerance, iterationsToAverage, leftCam,
               rightCam, leftWriter, rightWriter, orb, matcher, featureParams, objectDetectionParams, worldMap):
    numTotalIterations, consecutiveErrors, iterationCounter, iterationTime = 0, 0, 0, 0
    iterationTimes, cameraFTs, featureFTs, objectDectFTs, odometryFTs = list(), list(), list(), list(), list()
    leftImage, rightImage, grayLeftImage = None, None, None
    leftPts, leftKp, leftDesc, leftMatches = None, None, None, None
    objectBoundingBoxes = None
    while True:
        iterationStartTime = time.perf_counter()
        if LOG_ITERATION_INFO:
            Logger.log(f"#{numTotalIterations}: Started @ {iterationStartTime}")
        try:
            # save previous feature information
            prevGrayLeftImage = grayLeftImage
            prevLeftPts = leftPts
            prevLeftKp = leftKp
            prevLeftDesc = leftDesc
            prevLeftMatches = leftMatches
            prevObjectBoundingBoxes = objectBoundingBoxes

            # acquire previous segmented image
            segmentedImage = PayloadManager.getOutput('clustering', timeout=0.001)  # wait ~0, most likely empty

            cameraStartTime = time.perf_counter()
            # Satisfies that read images stage of control flow
            # Uncropped Images are needed for computing disparity; everything else should use
            # the cropped images which only contain pixels where the unditortion/rectification
            # map is valid. In general leftImage should be considered the primary image source.
            uncroppedLeftImage, uncroppedRightImage, leftImage, rightImage, grayLeftImage, _ = \
                fetchAndShowCameras(leftCam, rightCam, show=not HEADLESS, threadedDisplay=THREADED_DISPLAY)

            PayloadManager.addInputs('updateOdometer', [uncroppedLeftImage, uncroppedRightImage])
            cameraFTs.append(time.perf_counter() - cameraStartTime)

            featureStartTime = time.perf_counter()
            # feature points for left image
            # the point at index [0], [1], [2], etc. in both is the same real life feature
            # COMPUTES MATCHING FEATURES ACROSS BOTH CURRENT IMAGES
            # TODO since we aren't using this for visual odometry, if we don't need the feature matches
            # for anything else, we can reduce this method to only detect features and save computation time
            prevLeftPts, leftPts, prevLeftKp, prevLeftDesc, leftKp, leftDesc, leftMatches = \
                computeMatchingPoints(prevGrayLeftImage, grayLeftImage, orb, matcher, prevLeftKp, prevLeftDesc,
                                      ratio=featureParams['startingRatio'],
                                      featureRatio=featureParams["featureRatio"], stepSize=featureParams["stepSize"],
                                      timeout=featureParams["timeout"], show=not HEADLESS,
                                      threadedDisplay=THREADED_DISPLAY, windowName="LeftCaptures Matched Features")
            featureFTs.append(time.perf_counter() - featureStartTime)

            objectDectStartTime = time.perf_counter()
            # acquires the bounding box cordinates for areas of the image where there are dense features
            objectBoundingBoxes = objectDetection(leftImage, getPointsFromKeypoints(leftKp),
                                                  binSize=objectDetectionParams["binSize"],
                                                  featuresPerPixel=objectDetectionParams["featuresPerPixel"],
                                                  percentAreaThreshold=objectDetectionParams["percentAreaThreshold"],
                                                  connectedFeaturesThresh=objectDetectionParams["connFeaturesThresh"],
                                                  simplifyFinalOutput=True, show=not HEADLESS,
                                                  threadedDisplay=THREADED_DISPLAY)
            objectDectFTs.append(time.perf_counter() - objectDectStartTime)

            odometryStartTime = time.perf_counter()
            currentPose = PayloadManager.getOutput('updateOdometer')
            im3d = PayloadManager.getOutput('updateOdometer')
            odometryFTs.append(time.perf_counter() - odometryStartTime)

            # get current x, y position
            x, z = float(currentPose[0, 3]), float(currentPose[2, 3])
            current_node = worldMap.poseToNode(x, z)

            PayloadManager.addInputs('clustering', [leftImage, im3d])

            # route = astar(worldMap.get_grid(), weight=2.0)

            # all additional functionality should be present within the === comments
            # additional data that needs to be stored for each iteration should be handled above
            # ===========================================================================================================
            # TODO
            # Fill in remainder of functionality
            # stackedImage = experimental.combineImages(leftImage, im3d)
            # experimental.segmentImage(leftImage, im3d, K=5, iterations=1, show=not HEADLESS, threadedDisplay=THREADED_DISPLAY)

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
            odometryTimeStr = f", Avg odometry map: {getAvgTimeArr(odometryFTs, iterationCounter)} ms"
            Logger.log(iterNum + iterTimeStr + cameraTimeStr + featureTimeStr + objectDectTimeStr + odometryTimeStr)
            iterationCounter = 0
            iterationTimes, cameraFTs, featureFTs, objectDectFTs, odometryFTs = list(), list(), list(), list(), list()
            CaptureManager.updateAllFPS(1000.0 / avgIterTime, delayOffset=0.0)
            if not HEADLESS:
                DisplayManager.updateAllFPS(1000.0 / avgIterTime)
        numTotalIterations += 1
