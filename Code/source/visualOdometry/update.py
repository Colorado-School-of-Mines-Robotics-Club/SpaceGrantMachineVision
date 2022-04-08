# Libraries
import numpy as np
import cv2
from openVO import drawPoseOnImage
from typing import Tuple

# Custom imports
try:
    from source.cameras import DisplayManager
except ModuleNotFoundError:
    from Code.source.cameras import DisplayManager

def updateOdometer(args: Tuple) -> Tuple[np.ndarray, np.ndarray]:
    queue, odometer, show, td = args
    left = queue.getInput()
    right = queue.getInput()
    odometer.update(left, right)
    pose = odometer.current_pose()
    if show:
        disparity = cv2.normalize(odometer.current_disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        dispBGR = cv2.cvtColor(disparity, cv2.COLOR_GRAY2BGR)
        drawPoseOnImage(pose, dispBGR)
        if td:
            DisplayManager.show("Disparity map", dispBGR)
        else:
            cv2.imshow("Disparity map", dispBGR)
        cv2.waitKey(1)

    return pose, odometer.current_3d
