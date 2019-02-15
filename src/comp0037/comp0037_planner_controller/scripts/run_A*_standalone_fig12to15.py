#! /usr/bin/env python

from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.astar_always0_planner import ASTAR0Planner
from comp0037_planner_controller.astar_alwaysC_planner import ASTARCPlanner
from comp0037_planner_controller.astar_L2_planner import ASTARL2Planner
from comp0037_planner_controller.astar_octile_planner import ASTAROCTILEPlanner
from comp0037_planner_controller.astar_manhatten_planner import ASTARMANPlanner

occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(1, 19):
    occupancyGrid.setCell(11, y, 1)

start = (3, 18)
goal = (20, 0)

C_val=[0,5,10,50]

for C in C_val:

	title = 'A* C = ' + str(C)
	# *** Leave only the planner you want to use uncommented. ***

	#planner = ASTAR0Planner('A* 0', occupancyGrid);
	planner = ASTARCPlanner(title, occupancyGrid);
	#planner = ASTARL2Planner('A* Euclidean', occupancyGrid);
	#planner = ASTAROCTILEPlanner('A* Octile', occupancyGrid);
	#planner = ASTARMANPlanner('A* Manhatten', occupancyGrid);

	# *** Leave only the planner you want to use uncommented. ***
	planner.C = C
	planner.setRunInteractively(True)
	planner.setWindowHeightInPixels(400)
	goalReached = planner.search(start, goal)
	path = planner.extractPathToGoal()
