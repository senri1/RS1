# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque

# This class implements the A* planning algorithm with a constant 'C' cost to goal value.
# 


class ASTARCPlanner(CellBasedForwardSearch):

    # Define C the cost to goal as class variable.
    C = 1

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.astarCQueue = []

    # Add cell to list, then sort the list based on the cost to come + cost to goal value
    def pushCellOntoQueue(self, cell):
	self.astarCQueue.append(cell)
	if(cell.parent != None):
		cell.pathCost = cell.parent.pathCost + self.computeLStageAdditiveCost(cell.parent,cell) + self.C 
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
	predicted_path_cost = parentCell.pathCost + self.computeLStageAdditiveCost(parentCell,cell) + self.C
	if(predicted_path_cost < cell.pathCost):
		cell.parent = parentCell
		cell.pathCost = predicted_path_cost
		self.astarCQueue.sort(key = self.distance)        

    # Return the cost to come + cost to goal to use for sorting queue 	
    def distance(self,cell):
	return cell.pathCost 
		

