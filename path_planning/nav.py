# nav.py
import numpy as np
import math


class Map_Model(object):
	"""INSTANCE ATTRIBUTES:
	bike      [Bike object]
	waypoints [list]: list of of ordered points (x,y) that can be used to define line segments
	obstacles [list]: list of triples (x, y, r) which represent the coordinates
					   of the obstacles and the radius """

	def __init__(self, bike, waypoints, obstacles, paths = []):
		""" Map initializer """

		self.bike = bike
		self.paths = paths
		self.waypoints = waypoints
		self.obstacles = obstacles

	def add_path(self, p1, p2):
		self.paths.append([p1,p2])

	def get_path_vector(self, path_index):
		point1 = self.paths[path_index][0]
		point2 = self.paths[path_index][1]
		return np.array([point2[0]-point1[0], point2[1]-point1[1]])



class Nav(object):
	"""INSTANCE ATTRIBUTES:
		map [Map object] """


	def __init__(self, map_model):
		""" Nav initializer """
		self.map_model = map_model
		self.target_path = 0

	def path_length(self, point1, point2):
		""" point1 and point2 are the two point tuples that define a line segment
		This method finds the distance of this line segment """
		return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)		

	# def path_angle(point1, point2):
	# 	length = path_length(point1, point2)
	# 	angle = math.acos((point1[0]-point2[0])/length)
	# 	return angle


	def angle_from_path(self, point1, point2):
		""" [angle_from_path] returns the angle that the bicycle has to turn to 
		be perpendicular to the line defined by the tuple line_points as well as if
		it has to turn clockwise or counterclockwise """
		bike_vector = (math.cos(self.map_model.bike.direction), math.sin(self.map_model.bike.direction))
		path_vector = (point2[0]-point1[0], point2[1]-point1[1])
		dot_product = bike_vector[0]*path_vector[0] + bike_vector[1]*path_vector[1]
		
		angle = dot_product/abs(dot_product)*math.acos(dot_product/self.path_length(point1, point2))
		return np.degrees(angle)


	def direction_to_turn(self):

		bike_vector = np.array([math.cos(self.map_model.bike.direction), math.sin(self.map_model.bike.direction)])
		path_vector = self.map_model.get_path_vector(self.target_path)
		dot_product = np.sum(bike_vector*path_vector)
		return 0 if np.abs(dot_product)<.1 else dot_product/abs(dot_product)*(-1)




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
		self.h = 0.5
		self.r = 0.15

	@property
	def turn_rad():
		""" Calculate the turn radius (some function of velocity)"""


if __name__ == '__main__':
	import simulator
	new_bike = Bike((5,5), np.radians(80), .02)
	new_map = Map_Model(new_bike, [], [])
	new_map.add_path((5,0),(8,10))
	#new_map.add_path((0,1),(5.3,1))
	new_nav = Nav(new_map)
	sim = simulator.Simulator(new_map, new_nav)
	sim.run()

