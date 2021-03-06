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


def combineImages(image1: np.ndarray, image2: np.ndarray, ):
    return np.dstack([image1, image2])


# https://www.kdnuggets.com/2019/08/introduction-image-segmentation-k-means-clustering.html
def segmentImage(image: np.ndarray, image3d=None, method='minibatchkmeans', K=3, iterations=3, downscale=True,
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

    if image3d is not None:
        resized_image_3d = cv2.GaussianBlur(image3d, (3, 3), cv2.BORDER_DEFAULT)
        if downscale:
            width = int(image3d.shape[1] * downscaleRatio)
            height = int(image3d.shape[0] * downscaleRatio)
            dim = (width, height)
            resized_image_3d = cv2.resize(image3d, dim, resize_method_dict[downscaleMethod])

    resized_image = cv2.GaussianBlur(image, (3, 3), cv2.BORDER_DEFAULT)
    if downscale:
        width = int(image.shape[1] * downscaleRatio)
        height = int(image.shape[0] * downscaleRatio)
        dim = (width, height)
        resized_image = cv2.resize(image, dim, resize_method_dict[downscaleMethod])

    img = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
    if image3d is not None:
        combinedImage = np.dstack([img, resized_image_3d])
    else:
        combinedImage = img
    combinedImage = np.nan_to_num(combinedImage, neginf=0.0, posinf=0.0)

    _, _, channels = combinedImage.shape
    vectorized = np.float32(combinedImage.reshape((-1, channels)))

    cluster = cluster_method(n_clusters=K, n_init=iterations, random_state=0).fit(vectorized)
    centers, labels = cluster.cluster_centers_, cluster.labels_
    centers = np.uint8(centers)
    res = centers[labels.flatten()]
    res = res.reshape(combinedImage.shape)

    # resplit
    if image3d is not None:
        img = np.split(res, 2, axis=2)[0]
        img3d = np.split(res, 2, axis=2)[1]
    else:
        img = res

    result_image = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)

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


def runClustering(args: Tuple) -> Tuple:
    queue, show, td = args
    colorImage = queue.getInput()
    image3d = queue.getInput()

    result_image, centers, labels = segmentImage(colorImage, image3d, show=show, threadedDisplay=td)

    return result_image, centers, labels
