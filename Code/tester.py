from source.pathfinding.astar import astar
from source.pathfinding.pathfinding import plot_graph
from source.simulation.Map import Map
import numpy as np

test_grid = Map()

for i in range(100):
    test_grid.assign_rand()

obstacles = test_grid.get_grid()
passable_tiles = test_grid.get_passable()

print(type(passable_tiles[0][0]))

route = astar(obstacles, (0,0), (0,len(obstacles[0]) - 1), passable = passable_tiles)
plot_graph(obstacles, (0,0), (0, len(obstacles[0]) - 1), route)