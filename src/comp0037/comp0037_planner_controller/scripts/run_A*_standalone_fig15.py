#! /usr/bin/env python

from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.astar_always0_planner import ASTAR0Planner
from comp0037_planner_controller.astar_alwaysC_planner import ASTARCPlanner
from comp0037_planner_controller.astar_L2_planner import ASTARL2Planner
from comp0037_planner_controller.astar_octile_planner import ASTAROCTILEPlanner
from comp0037_planner_controller.astar_manhatten_planner import ASTARMANPlanner

occupancyGrid = OccupancyGrid(21, 21, 0.5)

#for y in xrange(1, 19):
#    occupancyGrid.setCell(11, y, 1)

start = (1, 1)
goal = (0, 20)

planner = ASTAROCTILEPlanner('A* Octile', occupancyGrid);

planner.setRunInteractively(True)
planner.setWindowHeightInPixels(400)
goalReached = planner.search(start, goal)
path = planner.extractPathToGoal()


