# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque

# This class implements the A* planning algorithm with the euclidean distance to the goal
# as the cost to goal value.


class ASTARL2Planner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.astarL2Queue = []

    # Add cell to list, then sort the list based on the cost to come + cost to goal value
    def pushCellOntoQueue(self, cell):
	self.astarL2Queue.append(cell)
	if(cell.parent != None):
		cell.pathCost = cell.parent.pathCost + self.computeLStageAdditiveCost(cell.parent,cell) + self.computeLStageAdditiveCost(self.goal,cell) 
	self.astarL2Queue.sort(key = self.distance)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.astarL2Queue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.astarL2Queue.pop(0)
        return cell

    # If a cell is visited again replace the previous parent cell with current 
    # parent cell if the cost to come + cost to goal is lower for the current parent.
    def resolveDuplicate(self, cell, parentCell):
	predicted_path_cost = parentCell.pathCost + self.computeLStageAdditiveCost(parentCell,cell) + self.computeLStageAdditiveCost(self.goal,cell) 
	if(predicted_path_cost < cell.pathCost):
		cell.parent = parentCell
		cell.pathCost = predicted_path_cost
		self.astarL2Queue.sort(key = self.distance)        

    # Return the cost to come + cost to goal to use for sorting queue 	
    def distance(self,cell):
	return cell.pathCost 
		

