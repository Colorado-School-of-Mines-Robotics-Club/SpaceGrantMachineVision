from source.pathfinding.astar import astar
from source.pathfinding.pathfinding import plot_graph
from source.simulation.Map import Map
import numpy as np

test_grid = Map()

for i in range(1000):
    test_grid.assign_rand()

obstacles = test_grid.get_grid()
passable_tiles = test_grid.get_passable()

route = astar(obstacles, (len(obstacles) - 1,len(obstacles[0])//2), (0,len(obstacles[0])//2), passable = passable_tiles)

if(route is not False):
    print(Map.instruction_converter(test_grid.DPerNode))
    print("\n")
    print(Map.instruction_converter(test_grid.DPerNode, compress = True))
    plot_graph(obstacles, (len(obstacles) - 1,len(obstacles[0])//2), (0,len(obstacles[0])//2), route)