# nav.py

class Map_Model(object):
	"""INSTANCE ATTRIBUTES:
	bike      [Bike object]
	waypoints [list]: list of of ordered points (x,y) that can be used to define line segments
	obstacles [list]: list of triples (x, y, r) which represent the coordinates
					   of the obstacles and the radius """

	def __init__(self, bike, waypoints, obstacles):
		""" Map initializer """

		self.bike = bike
		self.waypoints = waypoints
		self.obstacles = obstacles


class Nav(object):
	"""INSTANCE ATTRIBUTES:
		map [Map object] """

	def __init__(self, map_model):
		""" Nav initializer """
		self.map_model = map_model

    def nav():
    	""" Navigation algorithm """


class Bike(object):
	"""INSTANCE ATTRIBUTES:
	xy_coord  [tuple]: x and y coordinate of bicycle
	direction [float]: direction of bicycle relative to ......
	speed 	  [float]: speed of bicycle """

	def __init__(self, xy_coord, direction, speed):
		""" Bike initializer """
		self.xy_coord = xy_coord
		self.direction = direction
		self.speed = speed

	@property
	def turn_rad():
		""" Calculate the turn radius (some function of velocity)"""

