from source.pathfinding.astar import astar
from source.pathfinding.pathfinding import plot_graph
import numpy as np


grid = np.random.randint(0, 256, size=(30, 30))
#for i in range(len(grid)):
#    for j in range(len(grid[i])):
#        if(grid[i][j] > 1):
#            grid[i][j] = 0

route = astar(grid, (0,0), (len(grid) - 1, len(grid[0]) - 1), weight = 0.25)
plot_graph(grid, (0,0), (len(grid) - 1, len(grid[0]) - 1), route)