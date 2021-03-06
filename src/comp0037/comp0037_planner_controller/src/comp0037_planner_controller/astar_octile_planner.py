# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque
from math import *

# This class implements the A* planning algorithm with the octile distance to the goal
# as the cost to goal value.


class ASTAROCTILEPlanner(CellBasedForwardSearch):

    # Define the weight
    weight = 1

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.astarOCTILEQueue = []

    # Add cell to list, then sort the list based on the cost to come + cost to goal value
    def pushCellOntoQueue(self, cell):
	self.astarOCTILEQueue.append(cell)
	cell.costToGo = self.OCTILEdist(self.goal,cell)
	cell.pathCost = cell.costToGo
	if(cell.parent != None):
		cell.costToCome = cell.parent.costToCome + self.computeLStageAdditiveCost(cell.parent,cell)
		cell.pathCost =  cell.costToCome + self.weight * cell.costToGo
	self.astarOCTILEQueue.sort(key = self.distance)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.astarOCTILEQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.astarOCTILEQueue.pop(0)
        return cell

    # If a cell is visited again replace the previous parent cell with current 
    # parent cell if the cost to come + cost to goal is lower for the current parent.
    def resolveDuplicate(self, cell, parentCell):

	costToCome = parentCell.costToCome + self.computeLStageAdditiveCost(parentCell,cell)
	costToGo = self.OCTILEdist(self.goal,cell) 
	predicted_path_cost = costToCome + self.weight * costToGo 

	if(costToCome < cell.costToCome):
		cell.parent = parentCell
		cell.pathCost = predicted_path_cost
		cell.costToCome = costToCome
		self.astarOCTILEQueue.sort(key = self.distance)        

    # Return the cost to come + cost to goal to use for sorting queue 	
    def distance(self,cell):
	return cell.pathCost 


    def OCTILEdist(self,goalCell,cell):
	x = abs(cell.coords[0]-goalCell.coords[0])
	y = abs(cell.coords[1]-goalCell.coords[1])
	return max(x,y) + (sqrt(2)-1) * min(x,y)	
		

