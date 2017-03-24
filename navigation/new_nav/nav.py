# nav.py
import numpy as np
import math
import sys

class Nav(object):
	"""INSTANCE ATTRIBUTES:
		map [Map object] """


	def __init__(self, map_model):
		""" Nav initializer """
		self.map_model = map_model
		self.target_path = 0


	def direction_to_turn(self):
		""""""
		self.target_path = self.find_closest_path()
		distance = np.abs(self.distance_from_path())
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
		for path_index in range(len(self.map_model.paths)):
			bike_position = (self.map_model.bike.xB, self.map_model.bike.yB)
			nearest_point = geometry.nearest_point_on_path(self.map_model.paths[path_index], bike_position)
			distance_to_bike = geometry.dist(nearest_point, bike_position)
			if (closest_distance > distance_to_bike):
				closest_distance = distance_to_bike
				closest_path = path_index # CONTINUE HERE
		disp_next = self.displacement_to_turn(target_path = (closest_path+1)%len(self.map_model.paths))
		distance_next = self.distance_from_path((closest_path+1)%len(self.map_model.paths))
		if disp_next - np.abs(distance_next)>-0.01:
			closest_path = np.mod(closest_path + 1,len(self.map_model.paths))
		return closest_path