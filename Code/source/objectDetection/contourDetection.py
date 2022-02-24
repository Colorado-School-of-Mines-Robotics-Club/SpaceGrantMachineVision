# Additional libs
from typing import List
import numpy as np
import cv2
from source.cameras.DisplayManager import DisplayManager
from source.utilities.boundingBoxes import cv2RectToNpBoxes, drawBoundingBoxes, simplifyBoundingBoxes


def generateContourImage(image: np.ndarray, show=True, threadedDisplay=False) -> (np.ndarray, np.ndarray):
    blank = np.zeros(image.shape, dtype='uint8')
    _, blur, thresh = preProcessImage(image)
    canny = cv2.Canny(blur, 125, 175)
    contours, hierarchies = cv2.findContours(canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(blank, contours, -1, (0, 255, 0), 1)
    if show:
        if threadedDisplay:
            DisplayManager.show("contours drawn", blank)
        else:
            cv2.imshow("contours drawn", blank)
    return blank, contours


# function to get moments, areas, bounding boxes, and enclosing circles given some contours
def mabbec(contours: np.ndarray) -> (List, List, List, List):
    moments, areas, bounding_boxes, enclosing_circle = list(), list(), list(), list()
    for contour in contours:
        moments.append(cv2.moments(contour))
        areas.append(cv2.contourArea(contour))
        bounding_boxes.append(cv2.boundingRect(contour))
        enclosing_circle.append(cv2.minEnclosingCircle(contour))
    return moments, areas, bounding_boxes, enclosing_circle


# function given an image, convert to grayscale and perform blurring, and thresholding
def preProcessImage(image: np.ndarray, gaussianKernelSize=(9, 9)) -> (np.ndarray, np.ndarray, np.ndarray):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, gaussianKernelSize, 2)
    _, thresholded_image = cv2.threshold(blurred_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return gray_image, blurred_image, thresholded_image


def getContourBoundingBoxes(image: np.ndarray, show=True, threadedDisplay=False) -> List:
    _, contours = generateContourImage(image, show=False, threadedDisplay=False)
    _, _, contourBoxes, _ = mabbec(contours)
    npContourBoxes = cv2RectToNpBoxes(contourBoxes)
    npContourBoxes = simplifyBoundingBoxes(npContourBoxes)

    if show:
        drawBoundingBoxes(image, npContourBoxes, windowName="Contour Bounding Boxes", show=show,
                          threadedDisplay=threadedDisplay)

    return npContourBoxes
