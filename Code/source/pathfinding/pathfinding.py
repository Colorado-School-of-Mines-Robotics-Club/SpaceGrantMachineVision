# Built in python libs
import sys
import os
from typing import Dict, List

# Additional libs
import cv2
import numpy as np

# Custom imports
from source.logger.Logger import Logger
from source.utilities.Config import Config
from .astar import *
import matplotlib.pyplot as plt


# Source for visualization functions:
# https://www.analytics-link.com/post/2018/09/14/applying-the-a-path-finding-algorithm-in-python-part-1-2d-square-grid
def plot_graph(grid, start, goal, path=None):
    fig, ax = plt.subplots(figsize=(12, 12))

    ax.imshow(grid, cmap=plt.cm.Dark2)

    ax.scatter(start[1], start[0], marker="*", color="yellow", s=200)

    ax.scatter(goal[1], goal[0], marker="*", color="red", s=200)

    if type(path) != bool:
        ax.plot(path[:, 1], path[:, 0], color="black")

    plt.show()
