import sys
import copy
import heapq
import time
import random
from collections import deque
from operator import itemgetter
import numpy as np

class Node:
	def __init__(self, x, y, parent, cost):
		self.x = 0
		self.y = 0
		self.parent = None
		self.cost = 0

# ===============================================================================
# Class: PriorityQueue
#  Purpose: A simplified PriorityQueue
#  Author: Dr. Brian Bennett
class PriorityQueue:
    def __init__(self):
        self.queue = []

    def set_priority(self, item, priority):
        for node in self.queue:
            if node[0] == item:
                self.queue.remove(node)
                break
        self.put(item, priority)

    def put(self, item, priority):
        node = [item, priority]
        self.queue.append(node)
        self.queue.sort(key=itemgetter(1))

    def get(self):
        if len(self.queue) == 0:
            return None
        return self.queue.pop(0)
        
    def empty(self):
        return len(self.queue) == 0

class Map:
	'''
		Constructor - reads Mapfile
	'''
	def __init__(self,Mapfile):
		infile = open(Mapfile,'r')
		self.map = [list(row) for row in infile.read().splitlines()]
		self.search = copy.deepcopy(self.map)
		infile.close()
		self.set_start_goal()

	'''
		Searches the Map for a start location and a goal location
	'''
	def set_start_goal(self):
		r = 0
		c = 0
		for row in self.map:
			c = 0
			for val in row:
				if (val == 'S'):
					self.start = [r,c]
				elif (val == 'G'):
					self.goal = [r,c]
				c = c + 1
			r = r + 1

	'''
		Returns a list of all navigable (reachable) neighbors from the
		 provided location. You can move left, right, up, down, or diagonally--
		 assuming no obstacle is in the way.
	'''
	def get_neighbors(self,location):

		neighbors = []

		rows = np.arange(location[0]-1,location[0]+2)
		rows = rows[rows >= 0]
		rows = rows[rows <= 20]
		cols = np.arange(location[1]-1,location[1]+2)
		cols = cols[cols >= 0]
		cols = cols[cols <= 60]

		all_neighbors = np.transpose([np.tile(rows, len(cols)), np.repeat(cols, len(rows))])

		for loc in all_neighbors:
			if self.map[loc[0]][loc[1]] == "G" or self.map[loc[0]][loc[1]] == " ":
				neighbors.append([loc[0],loc[1]])
		if location in neighbors:
			neighbors.remove(location)
		return (neighbors)

	'''
		This returns a "guess" (heuristic) between location1 and location2.
		It should return a numeric value.
	'''
	def get_heuristic(self,location1,location2):
		
		# TODO: Implement heuristic function here
		return abs(location1[0] - location2[0] + abs(location1[1] - location2[1]))
		# return 0 # Return the heuristic value

	'''
		Returns a string containing the Map and start/end locations.
	'''
	def to_s(self):
		out = ""
		for row in self.map:
			for item in row:
				out = out + item
			out = out + "\n"
		out = out + "START LOCATION: " + str(self.start) + "\n"
		out = out + "GOAL LOCATION : " + str(self.goal) + "\n"
		out = out + "HEURISTIC DISTANCE BETWEEN START AND GOAL: " + str(self.get_heuristic(self.start,self.goal)) + "\n"
		return out

	'''
		Returns a string containing the Map as it looks during/after the search.
	'''
	def to_s_search(self):
		out = ""
		for row in self.search:
			for item in row:
				out = out + item
			out = out + "\n"
		return out

	'''
		Re-copies the Map into search, resetting any previous changes
	'''
	def reset_search(self):
		self.search = copy.deepcopy(self.map)

	'''
		Places the final path into the search variable.
		Requires explored parameter to contain the list of explored locations and the node
		 parameter to be the location at the end of the path.
	'''
	def backtrack(self,explored,node):
		path_cost = 0
		location = node[0]
		parent = node[1]
		# step_cost = node[2]
		# path_cost = path_cost + step_cost

		while (location != self.start):
			self.search[location[0]][location[1]] = 'O'
			newnode = [x for x in explored if x[0] == parent][0]
			location = newnode[0]
			parent = newnode[1]
			# step_cost = newnode[2]
			#path_cost = path_cost + step_cost
			path_cost += 1
		self.search[location[0]][location[1]] = 'O'
		return path_cost+1

	'''
	Breadth-First Search:
		This method implements a breadth-first search algorithm.
	'''
	def breadth_first_search(self):
		node = [self.start,[],0]	# The initial node has the [row,col] coordinate followed by its parent node and its step-cost
		frontier = deque()			# The double-ended queue structure representing the frontier
		frontier.appendleft(node)	# Add the start location to the open list.
		explored = []				# Create the closed (visited) list.

		while True:
			if len(frontier) == 0:
				return -1 # no path was able to be found

			current = frontier.pop()
			location, parent, cost = current

			# mark the node as explored (".") and add to explored list
			self.search[location[0]][location[1]] = '.'
			explored.append(current)

			# for every neighbor (row,col) relative to the current location
			for neighbor in self.get_neighbors(location):
				frontier_locations = [n[0] for n in frontier]
				explored_locations = [n[0] for n in explored]
				if neighbor not in frontier_locations and neighbor not in explored_locations:
					# if the neighbor would be the goal
					if neighbor == self.goal:
						# return the backtrack, passing in the goal node's location, parent, and cost
						return self.backtrack(explored, current)
					
					# append a new node to the frontier, passing in the neighbor's location, parent, and cost
					frontier.appendleft([neighbor,location,cost+1])

		return self.backtrack(closed,node)	# Return the path from the current node to the start node

	'''
	Depth-First Search:
		This method implements a depth-first search, a modified form of the
		 breadth-first search algorithm in the textbook.
	'''
	def depth_first_search(self):
		node = [self.start, [], 0]   # The initial node has the [row,col] coordinate, parent node, and step-cost
		frontier = deque()           # The double-ended queue structure representing the frontier
		frontier.append(node)        # Add the start location to the frontier (open list)
		explored = []                # Create the closed (visited) list

		while True:
			if len(frontier) == 0:
				return -1 

			# won't use appendleft() on the frontier, so this will act like
			# a stack (LIFO) and pop() and append() can be used 
			current = frontier.pop()
			location, parent, cost = current

			self.search[location[0]][location[1]] = '.'
			explored.append(current)

			for neighbor in self.get_neighbors(location):

				frontier_locations = [n[0] for n in frontier]
				explored_locations = [n[0] for n in explored]

				if neighbor not in frontier_locations and neighbor not in explored_locations:
					if neighbor == self.goal:
						return self.backtrack(explored, current)
					node_to_add = [neighbor, location, cost + 1]
					frontier.append(node_to_add)

		return self.backtrack(explored, node)  # Return the path from the current node to the start node

	"""
	Testing Uniform Cost Search with a PriorityQueue data structure.
	The runtime seems to be off on the search algorithms, but this didn't fix that.
	"""
	def uniform_cost_search_priority_queue(self):
		node = [self.start,[],0]
		frontier = PriorityQueue()
		frontier.put(node, 0) # root node
		explored = []

		while True:
			if len(frontier.queue) == 0:
				return -1
			
			current = frontier.get()
			location, parent, cost = current[0]

			self.search[location[0]][location[1]] = '.'
			explored.append(current[0])

			for neighbor in self.get_neighbors(location):
				
				frontier_locations = [n[0] for n in frontier.queue]
				explored_locations = [n[0] for n in explored]

				if neighbor not in frontier_locations and neighbor not in explored_locations:
					if neighbor == self.goal:
						return self.backtrack(explored, current[0])
					node_to_add = [neighbor, location, cost + 1]
					frontier.put(node_to_add, cost + 1)

		return self.backtrack(explored,node)	# Return the path from the current node to the start node


	'''
	Uniform-Cost Search:
		This method implements a uniform-cost search algorithm.
	'''
	def uniform_cost_search(self):
		node = [self.start,[],0]			# The initial node has the [row,col] coordinate followed by its parent node and its step-cost
		frontier = []
		heapq.heappush(frontier,(0,node))	# frontier becomes a min-heap and a tuple representing the cost of the node and the node are provided
		explored = []						# Create the closed (visited) list.

		#TODO: Code UCS here.
		while True:
			if len(frontier) == 0:
				return -1
			
			cost_so_far, current = heapq.heappop(frontier)
			location, parent, cost = current
	
			self.search[location[0]][location[1]] = '.'
			explored.append(current)

			for neighbor in self.get_neighbors(location):
				if neighbor == self.goal:
					return self.backtrack(explored, current)

				if 	all(not np.array_equal(neighbor,n[0]) for _,n in frontier) \
				and all(not np.array_equal(neighbor,n[0]) for n in explored):
					node_to_add = [neighbor, location, cost + 1]
					heapq.heappush(frontier, (cost + 1, node_to_add))
		return self.backtrack(explored,node)	# Return the path from the current node to the start node

'''
main method
	Creates the Map object by reading from the file specified on the command line.

	Runs the graph search, breadth-first search, depth-first search, and a-star search methods on the Map
	 and displays timing statistics for each.
'''
def main():
	filename = sys.argv[1]			# The file path is the first argument.
	map = Map(filename)
	print("Reading from file: %s\n" % filename)
	print(map.to_s())

	### Breadth-First Search
	bfs_start = time.process_time()
	path_cost = map.breadth_first_search()
	bfs_end = time.process_time()
	bfs_time = bfs_end - bfs_start
	print ("\n\nBREADTH-FIRST SOLUTION")
	print (map.to_s_search())
	print (" Breadth-First Path Cost: " + str(path_cost))
	map.reset_search()

	### Depth-First Search
	dfs_start = time.process_time()
	path_cost = map.depth_first_search()
	dfs_end = time.process_time()
	dfs_time = dfs_end - dfs_start
	print ("\n\nDEPTH-FIRST SOLUTION")
	print (map.to_s_search())
	print (" Depth-First Path Cost: " + str(path_cost))
	map.reset_search()
	
	### Uniform-Cost Search
	ucs_start = time.process_time()
	path_cost = map.uniform_cost_search()
	ucs_end = time.process_time()
	ucs_time = ucs_end - ucs_start
	print ("\n\nUNIFORM COST SOLUTION")
	print (map.to_s_search())
	print (" Uniform-Cost Path Cost: " + str(path_cost))
	map.reset_search()


	print("\n\nTIMING STATISTICS")
	print("=========================================")
	print(" Breadth-First Search : %f" % bfs_time)
	print(" Depth-First Search   : %f" % dfs_time)
	print(" Uniform-Cost Search  : %f" % ucs_time)


if __name__ == "__main__":
	main()
