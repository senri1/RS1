#! /usr/bin/env python

import csv
from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.astar_always0_planner import ASTAR0Planner
from comp0037_planner_controller.astar_alwaysC_planner import ASTARCPlanner
from comp0037_planner_controller.astar_L2_planner import ASTARL2Planner
from comp0037_planner_controller.astar_octile_planner import ASTAROCTILEPlanner
from comp0037_planner_controller.astar_manhatten_planner import ASTARMANPlanner

# This script just collects data for different weight values with the manhatten heuristic

occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(0, 19):
    occupancyGrid.setCell(11, y, 1)

start = (3, 18)
goal = (20, 0)

with open('Astar_MANHATTENdata','wb') as myfile:
	
	for i in range(0,21):

		weight = float(i)/2
		title = 'A* MANHATTEN k = ' + str(weight)
		planner = ASTARMANPlanner(title, occupancyGrid);
		planner.weight = weight

		goalReached = planner.search(start, goal)
		path = planner.extractPathToGoal()
		
		data = [weight,path.travelCost,path.angleTurned,planner.numberOfCellsVisited]
		wr = csv.writer(myfile,quoting=csv.QUOTE_ALL)
		wr.writerow(data)
