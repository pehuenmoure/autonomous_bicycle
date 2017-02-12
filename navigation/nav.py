# nav.py
import numpy as np
import math
import sys


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
		""" Adds a new path from p1 to p2 at the end of the path list """
		self.paths.append([p1,p2])

	def get_path_vector(self, path_index):
		""" Returns the unit vector of the path with index path_index """
		point1 = self.paths[path_index][0]
		point2 = self.paths[path_index][1]
		v =  np.array([point2[0]-point1[0], point2[1]-point1[1]])
		return v/np.linalg.norm(v)

	def dist(self, point1, point2):
		""" Calculates the distance between two points """
		return np.sqrt(self.dist2(point1, point2))	

	def dist2(self, point1, point2):
		return (np.square(point1[0]-point2[0]) + np.square(point1[1]-point2[1]))	

	def find_nearest_point_on_path(self, path_index):
		""" Returns the nearest point to the bike on the path with index 
		path_index """
		bike_coord = self.bike.xy_coord
		point1 = self.paths[path_index][0]
		point2 = self.paths[path_index][1]
		dist = self.dist(point1, point2)
		
		d2 = self.dist2(point1, point2)
		t = ((bike_coord[0] - point1[0])*(point2[0]-point1[0]) + (bike_coord[1] - point1[1]) * (point2[1] - point1[1]))/d2
		if (t < 0):
			return point1
		elif (t > 1):
			return point2
		else:
			x = point1[0] + t * (point2[0]-point1[0])
			y = point1[1] + t * (point2[1]-point1[1])
			return (x, y)

	def find_closest_path(self):
		""" Finds and returns the closest path to the bike from the list of paths """
		closest_distance = sys.maxint
		closest_path = 0
		for path_index in range(len(self.paths)):
			nearest_point = self.find_nearest_point_on_path(path_index)
			distance_to_bike = self.dist(nearest_point, self.bike.xy_coord)
			if (closest_distance > distance_to_bike):
				closest_distance = distance_to_bike
				closest_path = path_index
		return closest_path


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
		return np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)		

	# def path_angle(point1, point2):
	# 	length = path_length(point1, point2)
	# 	angle = math.acos((point1[0]-point2[0])/length)
	# 	return angle

	def _sign(self, v):
		if v == 0:
			return 0
		else:
			return v/np.abs(v)

	def angle_from_path(self, point1, point2):
		""" [angle_from_path] returns the angle that the bicycle has to turn to 
		be perpendicular to the line defined by the tuple line_points as well as if
		it has to turn clockwise or counterclockwise """
		bike_vector = (math.cos(self.map_model.bike.direction), math.sin(self.map_model.bike.direction))
		path_vector = (point2[0]-point1[0], point2[1]-point1[1])
		dot_product = bike_vector[0]*path_vector[0] + bike_vector[1]*path_vector[1]
		
		angle = self._sign(dot_product)*math.acos(dot_product/self.path_length(point1, point2))
		return np.degrees(angle)

	def distance_from_path(self):
		""""""
		p1 = np.array(self.map_model.paths[self.target_path][0])
		v = self.map_model.get_path_vector(self.target_path)
		if v[0] == 0:
			return self.map_model.bike.xy_coord[0] - p1[0]
		v_perp = np.array([v[1], -1*v[0]])
		bike_coords = np.array(self.map_model.bike.xy_coord)
		r = p1 - bike_coords
		dist = np.sum(v_perp*r)
		return dist


	def direction_to_turn(self):
		self.target_path = self.map_model.find_closest_path()
		print self.target_path, "TARGET PATH"
		distance = self.distance_from_path()
		sign = self._sign(self.distance_from_path())
		distance = np.abs(distance)
		p = self.map_model.get_path_vector(self.target_path)
		b = self.map_model.bike.vector
		r = self.map_model.bike.turning_r
		delta = np.abs(self.displacement_to_turn())
		if distance>r:
			return self.turn_perp()
		else:
			if np.abs(np.sum(p*b))<.01:
					return self.turn_parallel()

			elif np.sum(p*b)<0:  #outside 90 degrees of path direction
				return self.turn_perp()
			else:
				p_perp = np.array([-p[1], p[0]])*sign
				if np.sum(p_perp*b)>0: #facing away from path
					return self.turn_perp()
				else:
					if delta<distance:
						return self.turn_perp()
					else:
						return self.turn_parallel()


	def displacement_to_turn(self):
		p = self.map_model.get_path_vector(self.target_path)
		b = self.map_model.bike.vector
		R = np.array([[p[0], p[1]], [-p[1], p[0]]])
		p_R = R.dot(p)
		b_R = R.dot(b)
		phi = (np.arccos(b_R[0])-np.pi/2)%(2*np.pi)
		r = self.map_model.bike.turning_r
		delta = r+r*np.sin(phi)
		return delta


	def turn_parallel(self):
		path_vector = self.map_model.get_path_vector(self.target_path)
		path_perp = np.array([-path_vector[1], path_vector[0]])
		return self.turn_helper(path_perp)

	def turn_perp(self):
		path_vector = self.map_model.get_path_vector(self.target_path)
		p_orig = path_vector
		sign = self._sign(self.distance_from_path())
		path_vector = path_vector*sign
		return self.turn_helper(path_vector)

	def turn_helper(self, path_vector):
		bike_vector = self.map_model.bike.vector
		dot_product = np.sum(bike_vector*path_vector)
		return 0 if np.abs(dot_product)<.01 else self._sign(dot_product)*(-1)



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
		self.turning_r = 2

	@property
	def vector(self):
		b = np.array([math.cos(self.direction), math.sin(self.direction)])
		return b/np.linalg.norm(b)



if __name__ == '__main__':
	import simulator
	new_bike = Bike((1,1), np.radians(0), .02)
	new_map = Map_Model(new_bike, [], [])
	new_map.add_path((0,5),(5, 0))
	new_map.add_path((5,0),(10, 5))
	new_nav = Nav(new_map)
	sim = simulator.Simulator(new_map, new_nav)
	sim.run()

