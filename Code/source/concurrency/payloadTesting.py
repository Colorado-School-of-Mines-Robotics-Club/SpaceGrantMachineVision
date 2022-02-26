import cv2

from source.concurrency import PayloadManager
from source.cameras import CaptureManager, createCaptureSourceData
from source.visualOdometry import PTcomputeDisparity


def testPayloads():
    payloads = []
    payload1 = ("disparity", PTcomputeDisparity, (), 0.025)
    payloads.append(payload1)

    PayloadManager.initStart(payloads)

    leftCamera = cv2.VideoCapture(0)
    rightCamera = cv2.VideoCapture(1)

    while True:
        ok, left = leftCamera.read()
        ok, right = rightCamera.read()
        PayloadManager.addInputs('disparity', [left, right])
        disparity = PayloadManager.getOutput('disparity')
        cv2.imshow("disparity", disparity)
        cv2.waitKey(10)
