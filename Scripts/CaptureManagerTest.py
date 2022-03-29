import cv2
import sys


from Code.source.cameras import CaptureManager, createCaptureSourceData

if __name__ == '__main__' :

    source1 = createCaptureSourceData(0)
    source2 = createCaptureSourceData(1)
    CaptureManager.init([source1, source2])

    while True:
        cv2.imshow("source 1", CaptureManager.getFrame(0))
        cv2.imshow("source 2", CaptureManager.getFrame(1))

        keyPressed = cv2.waitKey(10) & 0xFF
        if keyPressed == 27:
            break