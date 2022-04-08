from source.pathfinding.astar import astar
from source.pathfinding.pathfinding import plot_graph
from source.simulation.Map import Map
import numpy as np

test_grid = Map()

for i in range(3000):
    test_grid.assign_rand()

obstacles = test_grid.get_grid()
passable_tiles = test_grid.get_passable()

route = astar(obstacles, (0,0), (len(obstacles) - 1,len(obstacles[0]) - 1), passable = passable_tiles)
plot_graph(obstacles, (0,0), (len(obstacles) - 1, len(obstacles[0]) - 1), route)

input("Hit enter for graph with passables!")

plot_graph(passable_tiles, (0,0), (len(obstacles) - 1, len(obstacles[0]) - 1), route)