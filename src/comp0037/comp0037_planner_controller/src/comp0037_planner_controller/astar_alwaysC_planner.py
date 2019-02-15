# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque

# This class implements the A* planning algorithm with a constant distance to the goal
# as the cost to goal value.


class ASTARCPlanner(CellBasedForwardSearch):

    # Define the weight
    C=1

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.astarCQueue = []

    # Add cell to list, then sort the list based on the cost to come + cost to goal value
    def pushCellOntoQueue(self, cell):
	self.astarCQueue.append(cell)
	cell.costToGo = self.C
	cell.pathCost = cell.costToGo
	if(cell.parent != None):
		cell.costToCome = cell.parent.costToCome + self.computeLStageAdditiveCost(cell.parent,cell) 
		cell.pathCost =  cell.costToCome +  cell.costToGo
	self.astarCQueue.sort(key = self.distance)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.astarCQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.astarCQueue.pop(0)
        return cell

    # If a cell is visited again replace the previous parent cell with current 
    # parent cell if the cost to come + cost to goal is lower for the current parent.
    def resolveDuplicate(self, cell, parentCell):

	costToCome = parentCell.costToCome + self.computeLStageAdditiveCost(parentCell,cell)
	costToGo = self.C
	predicted_path_cost = costToCome + costToGo

	if(costToCome < cell.costToCome):
		cell.parent = parentCell
		cell.pathCost = predicted_path_cost
		cell.costToCome = costToCome
		self.astarCQueue.sort(key = self.distance)        

    # Return the cost to come + cost to goal to use for sorting queue 	
    def distance(self,cell):
	return cell.pathCost 
		

