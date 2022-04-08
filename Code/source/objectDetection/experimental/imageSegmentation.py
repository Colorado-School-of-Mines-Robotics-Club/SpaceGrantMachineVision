# Built in python libs
import time
from typing import Tuple

# Additional libs
import cv2
import numpy as np
from sklearn.cluster import MiniBatchKMeans, KMeans, DBSCAN

# Custom  imports
try:
    from source.cameras.DisplayManager import DisplayManager
except ModuleNotFoundError:
    from Code.source.cameras.DisplayManager import DisplayManager


# https://www.kdnuggets.com/2019/08/introduction-image-segmentation-k-means-clustering.html
def segmentImage(image: np.ndarray, method='minibatchkmeans', K=3, iterations=3, downscale=True,
                 downscaleRatio=0.4, downscaleMethod='linear', show=False, threadedDisplay=False)\
        -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    cluster_method_dict = {
        'minibatchkmeans': MiniBatchKMeans,
        'kmeans': KMeans,
        'dbscan': DBSCAN
    }
    assert method in cluster_method_dict
    cluster_method = cluster_method_dict[method]

    resize_method_dict = {
        'nearest': cv2.INTER_NEAREST,
        'linear': cv2.INTER_LINEAR,
        'area': cv2.INTER_AREA,
        'cubic': cv2.INTER_CUBIC
    }
    assert downscaleMethod in resize_method_dict

    resized_image = cv2.GaussianBlur(image, (3, 3), cv2.BORDER_DEFAULT)
    if downscale:
        width = int(image.shape[1] * downscaleRatio)
        height = int(image.shape[0] * downscaleRatio)
        dim = (width, height)
        resized_image = cv2.resize(image, dim, resize_method_dict[downscaleMethod])

    img = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
    _, _, channels = img.shape
    vectorized = np.float32(img.reshape((-1, channels)))

    cluster = cluster_method(n_clusters=K, n_init=iterations, random_state=0).fit(vectorized)
    centers, labels = cluster.cluster_centers_, cluster.labels_
    centers = np.uint8(centers)
    res = centers[labels.flatten()]
    result_image = cv2.cvtColor(res.reshape(img.shape), cv2.COLOR_HSV2BGR)

    if downscale:
        width = int(image.shape[1])
        height = int(image.shape[0])
        dim = (width, height)
        result_image = cv2.resize(result_image, dim, resize_method_dict[downscaleMethod])

    if show:
        displayImg = np.concatenate((image, result_image), axis=1)
        if threadedDisplay:
            DisplayManager.show(f"segmentColors using {method}", displayImg)
        else:
            cv2.imshow(f"segmentColors using {method}", displayImg)

    return result_image, centers, labels
