# Built in python libs
import sys
import os
from typing import Dict, List, Callable
import heapq

# Additional libs
import cv2
import numpy as np

# Custom imports


'''
Euclidean Heuristic given starting and ending coordinates.
'''


def euclidean_heuristic(start, end):
    [x1, y1] = start
    [x2, y2] = end
    return ((x2-x1)**2 + (y2-y1)**2)**0.5


'''
AStar Algorithm

Input:
array - Array of 0s and 1s to run the pathfinding on
start - tuple of starting coordinates
goal - tuple of ending coordinates

Output: A numpy.ndarray of the nodes calculated

Source: https://www.analytics-link.com/post/2018/09/14/applying-the-a-path-finding-algorithm-in-python-part-1-2d-square-grid
'''


def astar(array, start, goal, heuristic: Callable):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == goal:
            data = []

            while current in came_from:
                data.append(current)
                current = came_from[current]

            return np.array(data)

        close_set.add(current)

        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue

                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False
