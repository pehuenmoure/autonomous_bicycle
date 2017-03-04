# nav.py
import numpy as np
import math
import sys
import requestHandler


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
		self.waypoints[0].append(p1[0])
		self.waypoints[0].append(p2[0])
		self.waypoints[1].append(p1[1])
		self.waypoints[1].append(p2[1])


	def add_point(self, p):
		# Changed to this to handle adding the first point (when there is no previous point)!!!
		if (len(self.paths) != 0):
			previous_point = self.paths[-1][1]
			print "ANOTHA ONE"
			self.paths.append([previous_point, p])
			print self.paths
		elif (len(self.waypoints[0])==1):
			#If the first point had been added we create the first path from the first point to p
			self.paths.append([(self.waypoints[0][0], self.waypoints[1][0]), p])
		self.waypoints[0].append(p[0])
		self.waypoints[1].append(p[1])


	def close_path(self):
		self.add_point(self.paths[0][0])


	def draw_circle(self, center, r, n_points, degrees = 2*np.pi):
		deg_inc = float(degrees)/n_points
		theta = deg_inc
		p0 = np.array(center) + np.array([r, 0])
		p1 = np.array(center) + np.array([r*np.cos(theta), r*np.sin(theta)])
		self.add_path(p0, p1)
		for i in range(2, n_points+1):
			next_point = np.array(center) + np.array([r*np.cos(i*theta), r*np.sin(i*theta)])
			self.add_point(next_point)



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
		"""Calculates the square of the distance of the two points """
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

	# def find_closest_path(self):
	# 	""" Finds and returns the closest path to the bike from the list of paths """
	# 	closest_distance = sys.maxint
	# 	closest_path = 0
	# 	for path_index in range(len(self.paths)):
	# 		nearest_point = self.find_nearest_point_on_path(path_index)
	# 		distance_to_bike = self.dist(nearest_point, self.bike.xy_coord)
	# 		if (closest_distance > distance_to_bike):
	# 			closest_distance = distance_to_bike
	# 			closest_path = path_index
	# 	desired_point = self.paths[closest_path][1]
	# 	if self.dist(desired_point, self.bike.xy_coord) < self.bike.turning_r:
	# 		closest_path = np.mod(closest_path + 1,len(self.paths))
	# 	return closest_path


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

	def distance_from_path(self, target_path = None):
		""""""
		if target_path is None:
			target_path = self.target_path
		p1 = np.array(self.map_model.paths[target_path][0])
		v = self.map_model.get_path_vector(target_path)
		if v[0] == 0:
			sign = self._sign(v[1])
			return  sign*(p1[0] - self.map_model.bike.xy_coord[0])
		v_perp = np.array([v[1], -1*v[0]])
		bike_coords = np.array(self.map_model.bike.xy_coord)
		r = p1 - bike_coords
		dist = np.sum(v_perp*r)
		return dist


	def direction_to_turn(self):
		self.target_path = self.find_closest_path()
		distance = np.abs(self.distance_from_path())
		delta = np.abs(self.displacement_to_turn())
		if delta<distance:
			return self.turn_perp()
		else:
			return self.turn_parallel()


	def displacement_to_turn(self, b = None, target_path = None):
		if target_path is None:
			target_path = self.target_path
		p = self.map_model.get_path_vector(target_path)
		if b is None:
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
		turn = 0 if np.abs(dot_product)<.01 else self._sign(dot_product)*(-1)
		facing_away = np.sum(np.array([-path_vector[1], path_vector[0]])*bike_vector)>0
		if turn == 0 and facing_away:
			return 1
		return turn

	def find_closest_path(self):
		""" Finds and returns the closest path to the bike from the list of paths """
		closest_distance = sys.maxint
		closest_path = 0
		for path_index in range(len(self.map_model.paths)):
			nearest_point = self.map_model.find_nearest_point_on_path(path_index)
			distance_to_bike = self.map_model.dist(nearest_point, self.map_model.bike.xy_coord)
			if (closest_distance > distance_to_bike):
				closest_distance = distance_to_bike
				closest_path = path_index
		disp_next = self.displacement_to_turn(target_path = (closest_path+1)%len(self.map_model.paths))
		distance_next = self.distance_from_path((closest_path+1)%len(self.map_model.paths))
		if disp_next - np.abs(distance_next)>-0.01:
			closest_path = np.mod(closest_path + 1,len(self.map_model.paths))
		return closest_path



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
		self.h = 2# I have modifies these values 
		self.turning_r = 2

	@property
	def vector(self):
		b = np.array([math.cos(self.direction), math.sin(self.direction)])
		return b/np.linalg.norm(b)


# def getCommand

if __name__ == '__main__':
	import simulator

	
	points = requestHandler.parse_json()
	new_bike = Bike((10, 10), math.pi*2, 0.01)
	new_map = Map_Model(new_bike, [[],[]], [])
	# new_map.add_path(points[0],points[1])
	# new_map.draw_circle(center = (7,7), r = 5, n_points = 10, degrees = np.pi/4)
	for p in points:
		print "point 1,2,3", p
		print "paths are", new_map.paths
		new_map.add_point(p)
	
	# new_map.add_point((10,15))
	# new_map.add_point((15,15))
	# new_map.add_point((18,11))
	# new_map.close_path()
	# new_map.add_path((0,9),(10,9))
	new_nav = Nav(new_map)
	sim = simulator.Simulator(new_map, new_nav)
	sim.run()

