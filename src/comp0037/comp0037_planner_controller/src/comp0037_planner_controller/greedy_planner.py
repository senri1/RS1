# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque

# This class implements the greedy planning algorithm.
#
# It works by inserting cells into the queue and sorting them
# based on their distance to the goal. Cells are popped from the 
# left.

class GREEDYPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.greedyQueue = []

    # Add cell to list, then sort the list
    def pushCellOntoQueue(self, cell):
        	self.greedyQueue.append(cell) 
		self.greedyQueue.sort(key = self.dist2Goal)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.greedyQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.greedyQueue.pop(0)
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass

    # Return the distance to from cell to goal 	
    def dist2Goal(self,cell):
	return self.computeLStageAdditiveCost(self.goal,cell)
		

