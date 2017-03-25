# nav.py
import numpy as np
import math
import sys
import geometry

class Nav(object):
	"""INSTANCE ATTRIBUTES:
		map [Map object] """


	def __init__(self, map_model):
		""" Nav initializer """
		self.map_model = map_model
		self.target_path = 0


	def direction_to_turn(self):
		""" Returns: the steer command """
		self.target_path = self.find_closest_path()
		bike_pos = (self.map_model.bike.xB, self.map_model.bike.yB)
		distance = np.abs(geometry.distance_from_path(bike_pos, self.map_model.paths[self.target_path]))
		delta = np.abs(self.displacement_to_turn())
		delta = delta + 10 # VALUE I calculated using my overshoot calculation functions
		if delta<distance:
			return self.turn_perp()
		else:
			return self.turn_parallel()


	def find_closest_path(self):
		""" Finds and returns the closest path to the bike from the list of paths """
		closest_distance = sys.maxint
		closest_path = 0
		bike_position = (self.map_model.bike.xB, self.map_model.bike.yB)
		for path_index in range(len(self.map_model.paths)):
			nearest_point = geometry.nearest_point_on_path(self.map_model.paths[path_index], bike_position)
			distance_to_bike = geometry.dist(nearest_point, bike_position)
			if (closest_distance > distance_to_bike):
				closest_distance = distance_to_bike
				closest_path = path_index 
		disp_next = self.displacement_to_turn(target_path = (closest_path+1)%len(self.map_model.paths))
		target_path = (closest_path+1)%len(self.map_model.paths)
		distance_next = geometry.distance_from_path(bike_position, self.map_model.paths(target_path))
		if disp_next - np.abs(distance_next)>-0.01:
			closest_path = np.mod(closest_path + 1,len(self.map_model.paths))
		return closest_path



	def displacement_to_turn(self, b = None, target_path = None):
		""" Returns: the delta which represents the distance at which the bike 
		should enter the s curve based on its turning radius """
		if target_path is None:
			target_path = self.target_path
		p = geometry.unit_vector(self.map_model.paths[target_path])
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
		""""""
		path_vector = geometry.unit_vector(self.map_model.paths[self.target_path])
		path_perp = np.array([-path_vector[1], path_vector[0]])
		return self.turn_helper(path_perp)


	def turn_perp(self):
		""""""
		path_vector = geometry.unit_vector(self.map_model.paths[self.target_path])
		p_orig = path_vector
		bike_pos = (self.map_model.bike.xB, self.map_model.bike.yB)
		sign = geometry.get_sign(geometry.distance_from_path(bike_pos, self.map_model.paths[self.target_path]))
		path_vector = path_vector*sign
		return self.turn_helper(path_vector)


	def turn_helper(self, path_vector):
		""""""
		bike_vector = self.map_model.bike.vector
		dot_product = np.sum(bike_vector*path_vector)
		turn = 0 if np.abs(dot_product)<.01 else geometry.get_sign(dot_product)*(-1)
		facing_away = np.sum(np.array([-path_vector[1], path_vector[0]])*bike_vector)>0
		if turn == 0 and facing_away:
			return 1
		return turn




