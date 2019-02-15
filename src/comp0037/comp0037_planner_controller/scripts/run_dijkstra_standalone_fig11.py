#! /usr/bin/env python


from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.dijkstra_planner import DIJKSTRAPlanner

occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(0, 20):
	occupancyGrid.setCell(11, y, 1)

start = (3, 18)
goal = (20, 0)

planner = DIJKSTRAPlanner('Dijkstra', occupancyGrid);
planner.setRunInteractively(True)

planner.setWindowHeightInPixels(400)

goalReached = planner.search(start, goal)

path = planner.extractPathToGoal()
#planner.extractPathEndingAtCoord((20,10))
#planner.extractPathEndingAtCoord((2,10))
#planner.extractPathEndingAtCoord((8,13))

