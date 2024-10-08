from graphics import *
from collections import deque
import math
from collections import deque
from operator import itemgetter
import numpy as np


# Use the ETSU-Official Colors - GO BUCS!
etsu_blue = color_rgb(4, 30, 68)
etsu_gold = color_rgb(255, 199, 44)
etsu_bg = color_rgb(223, 209, 167)




class PriorityQueue:
	"""
		Simple Priority Queue Class using a Python list
	"""
	def __init__(self):
		self.queue = []

	def put(self, item, priority):
		'''
			Add the item and sort by priority
		'''
		node = [item,priority]
		self.queue.append(node)
		self.queue.sort(key=itemgetter(1))

	def get(self):
		'''
			Return the highest-priority item in the queue
		'''
		if len(self.queue) == 0:
			return None
		node = self.queue.pop(0)
		return node[0]

	def empty(self):
		'''
			Return True if the queue has no items
		'''
		return len(self.queue) == 0



class Field:
	"""
		Class Field uses the graphics.py library

		It simulates an x by y field that contains Polygons, Lines, and Points.

		Search Space: Vertexes of each polygon, Start & End locations
	"""

	def __init__(self, width, height, intitle):
		'''
			Create lists of points, polygons
		'''
		self.points = []
		self.path = []
		self.polygons = []
		self.extras = []
		self.width = width
		self.height = height
		self.start = Point(0, 0)
		self.end = Point(0, 0)
		self.win = GraphWin(title=intitle,width=width,height=height)


	def setCoords(self, x1, y1, x2, y2):
		'''
			Set the viewport of the Field
		'''
		self.win.setCoords(x1, y1, x2, y2)


	def setBackground(self, color):
		'''
			Set the background color
		'''
		self.win.setBackground(color)


	def add_polygon(self, polygon):
		'''
			Add the polygon and the vertexes of the Polygon
		'''
		start = None
		self.points = self.points + polygon.getPoints()
		start = polygon.getPoints()[0]
		self.polygons.append(polygon)

		polygon.draw(self.win)


	def add_start(self, start):
		'''
			Add and display the starting location
		'''
		self.points.append(start)
		self.start = start
		c = Circle(start, 2)
		c.setFill('green')
		self.extras.append(c)
		text = Text(Point(start.x-8, start.y+10), 'Start')
		text.setSize(10)
		text.setTextColor('black')
		self.extras.append(text)
		text.draw(self.win)
		c.draw(self.win)


	def add_end(self, end):
		'''
			Add and display the ending location
		'''
		self.points.append(end)
		self.end = end
		c = Circle(end, 2)
		c.setFill('red')
		self.extras.append(c)
		text = Text(Point(end.x - 2, end.y - 10), 'End')
		text.setSize(10)
		text.setTextColor('black')
		self.extras.append(text)
		text.draw(self.win)
		c.draw(self.win)


	def get_neighbors(self, node):
		'''
		  Returns a list of neighbors of node -- Vertexes that the node can see.
		  All vertexes are within node's line-of-sight.
		'''
		neighbors = []

		# Loop through vertexes
		for point in self.points:
			# Ignore the vertex if it is the same as the node passed
			if (point == node):
				continue

			intersects = False

			# Create a line that represents a potential path segment
			pathSegment = Line(node, point)

			# Loop through the Polygons in the Field
			for o in self.polygons:
				# If the path segment intersects the Polygon, ignore it.
				if (o.intersects(pathSegment)):
					intersects = True
					break

			# If the path segment does not intersect the Polygon, it is a
			#  valid neighbor.
			if (not intersects):
				neighbors.append(point)

		return neighbors


	def wait(self):
		'''
			Pause the Window for action
		'''
		self.win.getMouse()


	def close(self):
		'''
			Closes the Window after a pause
		'''
		self.win.getMouse()
		self.win.close()


	def reset(self):
		for extra in self.extras:
			extra.undraw()
		self.extras = []


	def backtrack(self, came_from, node):
		'''
			Recreate the path located.

			Requires a came_from dictionary that contains the parents of each node.
			The node passed is the end of the path.
		'''
		current = node
		self.path.append(current)
		parent = came_from[str(current)]
		while parent != self.start:
			line = Line(current,parent)
			line.setOutline("green")
			line.setArrow("first")
			self.extras.append(line)
			line.draw(self.win)
			current = parent
			parent = came_from[str(current)]
			self.path.append(current)
		line = Line(current,parent)
		line.setOutline("green")
		line.setArrow("first")
		line.draw(self.win)
		self.extras.append(line)
		self.path.append(parent)
		self.path.reverse()


	def straight_line_distance(self, point1, point2):
		'''
			Returns the straight-line distance between point 1 and point 2
		'''
	# TODO: Calculate Straight-Line Distance
		return 0.


	def astar_search(self):
		'''
		Create the A* Search Here

			Use the Backtrack method to draw the final path when your
			 algorithm locates the end point
		'''
		node = self.start

		frontier = PriorityQueue()
		frontier.put(node,0)
		came_from = {}
		cost_so_far = {}

		came_from[str(node)] = None
		cost_so_far[str(node)] = 0

	# TODO: Continue A* Algorithm Here

		return cost_so_far[str(node)]



def main():
	f = Field(700, 400, "Search Space")
	f.setCoords(90, 500, 400, 700)
	f.setBackground(etsu_bg)

	# Setup Polygons
	p1 = Polygon(Point(100, 600), Point(200, 620), Point(100, 640))
	p1.setFill(etsu_blue)

	p2 = Polygon(Point(160, 550), Point(190, 570), Point(160, 590), Point(130, 570))
	p2.setFill(etsu_gold)

	p3 = Polygon(Point(205, 610), Point(250, 600), Point(240, 640), Point(210, 630))
	p3.setFill(etsu_blue)

	p4 = Polygon(Point(290, 550), Point(320, 570), Point(290, 610), Point(260, 570))
	p4.setFill(etsu_gold)

	p5 = Polygon(Point(340, 620), Point(370, 630), Point(360, 650), Point(330, 640))
	p5.setFill(etsu_blue)

	p6 = Polygon(Point(120, 520), Point(140, 540), Point(130, 560), Point(110, 550))
	p6.setFill(etsu_gold)

	p7 = Polygon(Point(170, 520), Point(200, 540), Point(180, 560))
	p7.setFill(etsu_blue)

	p8 = Polygon(Point(230, 520), Point(260, 530), Point(250, 550), Point(220, 540))
	p8.setFill(etsu_gold)

	p9 = Polygon(Point(340, 520), Point(360, 540), Point(350, 560), Point(330, 550))
	p9.setFill(etsu_blue)

	p10 = Polygon(Point(120, 670), Point(150, 690), Point(140, 695), Point(110, 690))
	p10.setFill(etsu_gold)

	p11 = Polygon(Point(170, 660), Point(200, 680), Point(190, 700), Point(160, 680))
	p11.setFill(etsu_blue)

	p12 = Polygon(Point(220, 670), Point(250, 690), Point(240, 695), Point(210, 690))
	p12.setFill(etsu_gold)

	p13 = Polygon(Point(270, 660), Point(300, 680), Point(290, 700), Point(260, 680))
	p13.setFill(etsu_blue)

	p14 = Polygon(Point(320, 650), Point(350, 690), Point(340, 680), Point(310, 690))
	p14.setFill(etsu_gold)

	p15 = Polygon(Point(340, 560), Point(370, 580), Point(360, 600), Point(330, 590), Point(320, 570))
	p15.setFill(etsu_blue)

	f.add_polygon(p1)
	f.add_polygon(p2)
	f.add_polygon(p3)
	f.add_polygon(p4)
	f.add_polygon(p5)
	f.add_polygon(p6)
	f.add_polygon(p7)
	f.add_polygon(p8)
	f.add_polygon(p9)
	f.add_polygon(p10)
	f.add_polygon(p11)
	f.add_polygon(p12)
	f.add_polygon(p13)
	f.add_polygon(p14)
	f.add_polygon(p15)

	'''
		Try BOTH SETS of Start and End Points
	'''
	f.add_start(Point(110, 650))
	f.add_end(Point(390, 540))


	path_cost = f.astar_search()


	print("Straight Line Distance from Start to Goal: %f" % f.straight_line_distance(f.start,f.end))
	print("Path Cost from Start to Goal: %f" % path_cost)
	print("Path----------")
	print(f.path)

	f.wait()	# Click to continue
	f.reset()	# Reset the Field

	f.add_start(Point(120, 640))
	f.add_end(Point(390, 510))

	path_cost = f.astar_search()

	print("Straight Line Distance from Start to Goal: %f" % f.straight_line_distance(f.start,f.end))
	print("Path Cost from Start to Goal: %f" % path_cost)
	print("Path----------")
	print(f.path)

	f.wait()	# Click to continue
	f.reset()   # Reset the Field

	# TODO: Create the Start and End points of your choice
	#f.add_start(Point(x, y))
	#f.add_end(Point(x, y))

	path_cost = f.astar_search()

	print("Straight Line Distance from Start to Goal: %f" % f.straight_line_distance(f.start,f.end))
	print("Path Cost from Start to Goal: %f" % path_cost)
	print("Path----------")
	print(f.path)

	f.close()

main()
