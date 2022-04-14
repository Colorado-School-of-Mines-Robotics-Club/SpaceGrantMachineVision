# Built in python libs
import sys
import os
from typing import Dict, List

# Additional libs
import cv2
import numpy as np

# Custom imports
import matplotlib.pyplot as plt
from bezier import Curve

# Source for visualization functions:
# https://www.analytics-link.com/post/2018/09/14/applying-the-a-path-finding-algorithm-in-python-part-1-2d-square-grid
def plot_graph(grid, start, goal, path=None):
    fig, ax = plt.subplots(figsize=(12, 12))

    ax.imshow(grid, cmap='Reds')

    ax.scatter(start[1], start[0], marker="*", color="blue", s=30)

    ax.scatter(goal[1], goal[0], marker="*", color="green", s=30)

    if type(path) != bool:
        ax.plot(path[:, 1], path[:, 0], color="black")

    plt.show()

def node_compress(route):
    operatives = [route[0]]
    if(route[0][0] == route[1][0]):
        change = 0
    else:
        change = 1

    for i in range(len(route) - 1):

        if(route[i][change] != route[i+1][change]):
            change = 1 - change
            operatives.append(route[i])
    
    return operatives

def big_bezzy(grid, start, goal,route,NUM_OF_POINTS = 100):
    #Input: route consisting of only corners
    #Output: The mother of all Bezier Curves

    curve = Curve(route.T, degree = len(route) - 1)
    points = [X/NUM_OF_POINTS for X in range(0,NUM_OF_POINTS + 1)]
    smoothened = [[curve.evaluate(i)[0][0],curve.evaluate(i)[1][0]] for i in points]

    return smoothened

def mini_bezzies(route):

    return None