#! /usr/bin/env python

from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.astar_always0_planner import ASTAR0Planner
from comp0037_planner_controller.astar_alwaysC_planner import ASTARCPlanner
from comp0037_planner_controller.astar_L2_planner import ASTARL2Planner
from comp0037_planner_controller.astar_octile_planner import ASTAROCTILEPlanner
from comp0037_planner_controller.astar_manhatten_planner import ASTARMANPlanner

occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(0, 19):
    occupancyGrid.setCell(11, y, 1)

start = (3, 18)
goal = (20, 0)

weight_val=[0.5,1,1.5,3]

for weight in weight_val:

	title = 'A* Euclidean k = ' + str(weight)
	# *** Leave only the planner you want to use uncommented. ***

	#planner = ASTAR0Planner('A* 0', occupancyGrid);
	#planner = ASTARCPlanner(title, occupancyGrid);
	planner = ASTARL2Planner(title, occupancyGrid);
	#planner = ASTAROCTILEPlanner('A* Octile', occupancyGrid);
	#planner = ASTARMANPlanner('A* Manhatten', occupancyGrid);

	# *** Leave only the planner you want to use uncommented. ***
	planner.weight = weight
	planner.setRunInteractively(True)
	planner.setWindowHeightInPixels(400)
	goalReached = planner.search(start, goal)
	path = planner.extractPathToGoal()
