# Built in python libs
from typing import List, Tuple

# Additional libs
import numpy as np
import cv2
from numba import jit, njit
from numba.typed import List as tList

# Custom  imports
from source.logger.Logger import Logger
from source.utilities import exceptions
from source.cameras import DisplayManager


# image is a cv2 image, which is a numpy array
# boundingBoxes is as follows
#        [ [x1, y1], [x2, y2] ]
#        where x1, y1, x2, y2 are any number
# the number type gets sanitized upon boundingBox load coordinates
def drawBoundingBoxes(rawImage: np.ndarray, boundingBoxes: List, color=(0, 0, 255), thickness=2,
                      windowName="Bounding Boxes", show=False, threadedDisplay=True) -> np.ndarray:
    image = np.copy(rawImage)
    for box in boundingBoxes:
        [p1, _, p3, _] = getBoundingBoxPoints(box)
        if isinstance(p1, np.ndarray) or isinstance(p3, np.ndarray):
            p1, p3 = (p1[0], p1[1]), (p3[0], p3[1])
        cv2.rectangle(image, p1, p3, color, thickness)
    if show:
        if threadedDisplay:
            DisplayManager.show(windowName, image)
        else:
            cv2.imshow(windowName, image)
    return image


@jit(nopython=True)
def cv2RectToNpBoxes(boundingBoxes):
    npBoxes = []
    for (x, y, w, h) in boundingBoxes:
        npBoxes.append(np.array([(x, y), (x + w, y + h)]).astype('int64'))
    return npBoxes


@jit(nopython=True)
def npToCv2RectBoxes(boundingBoxes):
    cv2RectBoxes = []
    for (x1, y1, x2, y2) in boundingBoxes:
        cv2RectBoxes.append((x1, y1, (x2 - x1), (y2 - y1)))
    return cv2RectBoxes


# gets the coordinates out of the bounding box list/array
# bounding box must be a np.array
# np.array([[x1, y1], [x2, y2]])
@jit(nopython=True)
def getBoundingBoxCords(box: np.ndarray) -> Tuple[int, int, int, int]:
    #      x1 [0]          y1 [1]          x2 [2]          y2 [3]
    return int(box[0][0]), int(box[0][1]), int(box[1][0]), int(box[1][1])


# makes points out of the bounding box coordinates
@jit(nopython=True)
def getBoundingBoxPoints(box: np.ndarray) -> np.ndarray:
    pts = getBoundingBoxCords(box)
    x1 = pts[0]
    y1 = pts[1]
    x2 = pts[2]
    y2 = pts[3]
    return np.array([(x1, y1), (x2, y1), (x2, y2), (x1, y2)]).astype('int64')


# checks each point in a boundingBox and determines they are equal if each point is equal
@jit(nopython=True)
def boundingBoxEquals(box1: np.ndarray, box2: np.ndarray) -> bool:
    if not (box1[0][0] == box2[0][0] and box1[0][1] == box2[0][1] and
            box1[1][0] == box2[1][0] and box1[1][1] == box2[1][1]):
        return False
    return True


# determine if there is a connection between two bounding boxes
@jit(nopython=True)
def determineConnection(box1: np.ndarray, box2: np.ndarray) -> bool:
    x11, y11, x12, y12 = getBoundingBoxCords(box1)
    x21, y21, x22, y22 = getBoundingBoxCords(box2)

    leftInside: bool = x11 <= x21 <= x12
    rightInside: bool = x11 <= x22 <= x12
    topInside: bool = y11 <= y21 <= y12
    bottomInside: bool = y11 <= y22 <= y12

    if (leftInside or rightInside) and (topInside or bottomInside):
        return True
    return False


# determines the new corners of the bounding box encapsulating two other bounding boxes
@jit(nopython=True)
def determineMaxMinCorners(boundingBoxes: List) -> np.ndarray:
    if len(boundingBoxes) == 1:
        return boundingBoxes[0].astype('int64')
    x1s = []
    y1s = []
    x2s = []
    y2s = []
    for box in boundingBoxes:
        x1, y1, x2, y2 = getBoundingBoxCords(box)
        x1s.append(x1)
        y1s.append(y1)
        x2s.append(x2)
        y2s.append(y2)
    minX = int(min(x1s))
    maxX = int(max(x2s))
    minY = int(min(y1s))
    maxY = int(max(y2s))
    # construct a new boundingBox
    return np.asarray([[minX, minY], [maxX, maxY]]).astype('int64')


# functions that given bounding box data combines connected bounding boxes
# @jit(forceobj=True)
def simplifyBoundingBoxes(boundingBoxes: List) -> List:
    if len(boundingBoxes) == 0:
        return [np.asarray([[0, 0], [0, 0]]).astype('int64')]
    if len(boundingBoxes) <= 1:
        return [boundingBoxes[0].astype('int64')]
    connectedBoxes = [boundingBoxes[0].astype('int64')]
    simplifiedBoxes = [boundingBoxes[0].astype('int64')]
    connectedBoxes.pop(0)
    simplifiedBoxes.pop(0)
    for i, box in enumerate(boundingBoxes):
        connectedBoxes.append(box.astype('int64'))
        for j, currBox in enumerate(connectedBoxes):
            for k, nextBox in enumerate(boundingBoxes):
                if not boundingBoxEquals(currBox, nextBox):
                    if determineConnection(currBox, nextBox):
                        connectedBoxes.append(nextBox.astype('int64'))
                        boundingBoxes.pop(k)
        simplifiedBoxes.append(determineMaxMinCorners(connectedBoxes))
        connectedBoxes = [np.asarray([[0, 0], [0, 0]]).astype('int64')]
        connectedBoxes.pop(0)
    return simplifiedBoxes
