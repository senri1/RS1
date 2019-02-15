#! /usr/bin/env python



from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.greedy_planner import GREEDYPlanner

occupancyGrid = OccupancyGrid(21, 21, 0.5)

#for y in xrange(1, 19):
#    occupancyGrid.setCell(11, y, 1)

start = (10, 10)
goal = (20, 10)

planner = GREEDYPlanner('Greedy Search', occupancyGrid);
planner.setRunInteractively(True)

planner.setWindowHeightInPixels(400)

goalReached = planner.search(start, goal)

path = planner.extractPathToGoal()
