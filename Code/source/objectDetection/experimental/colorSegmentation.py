# Built in python libs
import time

# Additional libs
import cv2
import numpy as np
from sklearn.cluster import DBSCAN

# Custom  imports
from source.cameras.DisplayManager import DisplayManager


# https://www.kdnuggets.com/2019/08/introduction-image-segmentation-k-means-clustering.html
def segmentColors(image: np.ndarray, method='kmeans', K=3, iterations=5, eps=0.5, min_samples=10,
                  downscale=True, downscaleRatio=0.5, downscaleMethod='linear', show=False, threadedDisplay=False)\
        -> np.ndarray:
    method_list = ['kmeans']
    assert method in method_list

    methods_dict = {
        'nearest': cv2.INTER_NEAREST,
        'linear': cv2.INTER_LINEAR,
        'area': cv2.INTER_AREA,
        'cubic': cv2.INTER_CUBIC
    }
    assert downscaleMethod in methods_dict

    resized_image = cv2.GaussianBlur(image, (3, 3), cv2.BORDER_DEFAULT)
    if downscale:
        width = int(image.shape[1] * downscaleRatio)
        height = int(image.shape[0] * downscaleRatio)
        dim = (width, height)
        resized_image = cv2.resize(image, dim, methods_dict[downscaleMethod])

    img = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
    vectorized = np.float32(img.reshape((-1, 3)))
    if method == 'kmeans':
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        ret, label, center = cv2.kmeans(vectorized, K, None, criteria, iterations, cv2.KMEANS_PP_CENTERS)
        center = np.uint8(center)
        res = center[label.flatten()]
        result_image = cv2.cvtColor(res.reshape(img.shape), cv2.COLOR_RGB2BGR)

    if downscale:
        width = int(image.shape[1])
        height = int(image.shape[0])
        dim = (width, height)
        result_image = cv2.resize(result_image, dim, methods_dict[downscaleMethod])

    if show:
        displayImg = np.concatenate((image, result_image), axis=1)
        if threadedDisplay:
            DisplayManager.show(f"segmentColors using {method}", displayImg)
        else:
            cv2.imshow(f"segmentColors using {method}", displayImg)

    return result_image
