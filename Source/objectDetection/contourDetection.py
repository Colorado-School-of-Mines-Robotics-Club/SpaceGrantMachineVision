# Additional libs
import numpy as np
import cv2

def displayContours(image, show=True, threadedDisplay=False):
    blank = np.zeros(image.shape, dtype='uint8')
    _, blur, thresh = preProcessImage(image)
    canny = cv2.Canny(blur, 125, 175)
    contours, hierarchies = cv2.findContours(canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(blank, contours, -1, (0, 255, 0), 1)
    if show == True:
        cv2.imshow("contours drawn", blank)
    return contours

# function to get moments, areas, bounding boxes, and enclosing circles given some contours
def mabbec(contours):
    moments, areas, bounding_boxes, enclosing_circle = list(), list(), list(), list()
    for contour in contours:
        moments.append(cv2.moments(contour))
        areas.append(cv2.contourArea(contour))
        bounding_boxes.append(cv2.boundingRect(contour))
        enclosing_circle.append(cv2.minEnclosingCircle(contour))
    return moments, areas, bounding_boxes, enclosing_circle

# function given an image, convert to grayscale and perform blurring, and thresholding
def preProcessImage(image, gaussianKernelSize=(9,9), show=False):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, gaussianKernelSize, 2)
    _, thresholded_image = cv2.threshold(blurred_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return gray_image, blurred_image, thresholded_image
