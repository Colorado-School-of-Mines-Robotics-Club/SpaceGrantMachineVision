# Built in python libs
import sys
import time

# Additional libs
import cv2
import numpy as np

# Custom imports
from .logger import Logger, logArguments, logSystemInfo, logConfiguration
from .cameras import fetchAndShowCameras, DisplayManager, CaptureManager
from .features import getImageKeyDesc, getPointsFromKeypoints
from .objectDetection import objectDetection, experimental
from .simulation import Map, Robot
from .utilities import getAvgTimeArr, exceptions, getBoxesXYZ
from .concurrency import PayloadManager
from .pathfinding import astar, plot_graph


def autonomous(HEADLESS, LOG_ITERATION_INFO, THREADED_DISPLAY, RECORD, VIDEO, errorTolerance, iterationsToAverage,
               leftCam, rightCam, leftWriter, rightWriter, orb, objectDetectionParams, worldMap, interface):
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
            if numTotalIterations == 59:
                print("should be a bug here")

            # save previous feature information
            prevGrayLeftImage = grayLeftImage
            prevLeftPts = leftPts
            prevLeftKp = leftKp
            prevLeftDesc = leftDesc
            prevLeftMatches = leftMatches
            prevObjectBoundingBoxes = objectBoundingBoxes

            # acquire previous segmented image
            segmentedImage = PayloadManager.getOutput('clustering', timeout=0.0)

            cameraStartTime = time.perf_counter()
            # Satisfies that read images stage of control flow
            # Uncropped Images are needed for computing disparity; everything else should use
            # the cropped images which only contain pixels where the unditortion/rectification
            # map is valid. In general leftImage should be considered the primary image source.
            # uncroppedLeftImage, uncroppedRightImage, leftImage, rightImage, grayLeftImage, _ = \
            #     fetchAndShowCameras(leftCam, rightCam, show=not HEADLESS, threadedDisplay=THREADED_DISPLAY)
            if VIDEO:
                leftFrameData, rightFrameData = PayloadManager.getOutput('cameras')
            else:
                frameData = PayloadManager.getOutputs('cameras')
                leftFrameData, rightFrameData = frameData[len(frameData) - 1]
            uncroppedLeftImage, leftImage, leftFrameTime = leftFrameData
            uncroppedRightImage, rightImage, rightFrameTime = rightFrameData

            PayloadManager.addInputs('updateOdometer', [uncroppedLeftImage, uncroppedRightImage])
            cameraFTs.append(time.perf_counter() - cameraStartTime)

            featureStartTime = time.perf_counter()
            # feature points for left image
            leftKp, leftDesc = getImageKeyDesc(leftImage, orb)
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
            currentPose, im3d = PayloadManager.getOutput('updateOdometer')
            # as of right now filter out all inf and -inf
            # TODO do this better?? in openVO most likely
            im3d = np.nan_to_num(im3d, neginf=0.0, posinf=0.0)
            odometryFTs.append(time.perf_counter() - odometryStartTime)

            PayloadManager.addInputs('clustering', [leftImage, im3d])

            # get the real world cordinates and update the map
            box_cords = getBoxesXYZ(im3d, objectBoundingBoxes)
            worldMap.incrementNodeScoresFromPose(box_cords)

            # get current x, y position
            x, z = float(currentPose[0, 3]), float(currentPose[2, 3])
            current_node = worldMap.poseToNode(x, z)

            route = astar(worldMap.get_grid(), current_node, worldMap.getEndNode(), weight=2.0,
                          passable=worldMap.get_passable())

            if not HEADLESS:
                # plot_graph(worldMap.get_grid(), current_node, worldMap.getEndNode(), route)
                if THREADED_DISPLAY:
                    DisplayManager.show("World Map", worldMap.draw())
                else:
                    cv2.imshow("World Map", worldMap.draw())

            if isinstance(route, np.ndarray):
                world_route = worldMap.convert_route_to_dist(route)
                # TODO update interface from world_route
                PayloadManager.addInputs('hardware', interface.getCommandPWM())
            else:
                PayloadManager.addInputs('hardware', [0 for i in range(16)])

            # TODO ??

            # ==========================================================================================================
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
