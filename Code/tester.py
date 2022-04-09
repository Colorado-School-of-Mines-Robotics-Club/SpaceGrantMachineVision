import os

# directory = os.getcwd()
# print(directory)
# os.chdir(directory + '\\source')
# print(os.getcwd())


from source.pathfinding.astar import astar
from source.pathfinding.pathfinding import plot_graph
from source.simulation.Map import Map
import numpy as np

test_grid = Map()

for i in range(1000):
    test_grid.assign_rand()

obstacles = test_grid.get_grid()
passable_tiles = test_grid.get_passable()

route = astar(obstacles, (len(obstacles) - 1, len(obstacles[0]) // 2), (0, len(obstacles[0]) // 2),
              passable=passable_tiles)

if route is not False:
    print(test_grid.instruction_converter(route))
    print("\n")
    print(test_grid.instruction_converter(route, compress=True))
    plot_graph(obstacles, (len(obstacles) - 1, len(obstacles[0]) // 2), (0, len(obstacles[0]) // 2), route)
