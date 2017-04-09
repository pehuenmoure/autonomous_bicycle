import nav
import mapModel
import math
import bikeState
import bikeSim
from constants import *
import geometry

import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib import collections  as mc

def main_loop(nav, bike):
	""" This is the main loop that gets the nav command, passes it through bike dynamics
	to get the updated state and plots the new state """
	k = 0 

	while (k < 700):

		#plotting
		plt.scatter(bike.xB, bike.yB)
		plt.show()
		plt.pause(0.00001)

		# Steer vs Time
		# plt.scatter((k+1)*TIMESTEP, bike.delta)
		# plt.show()
		# plt.pause(0.00001)


		steerD = nav.direction_to_turn()
		# print steerD
		# if new state has new target path then recalculate delta
		new_state = bikeSim.new_state(bike, steerD)
		bike = new_state
		nav.map_model.bike = bike
		# if k == 7:
			# print "HELLOOOO", nav.calc_overshoot()
			# print "HELLOOOO", nav.calc_overshoot()
		path_angle = geometry.line_angle(nav.map_model.paths[nav.target_path])
		bike_angle = nav.map_model.bike.psi

		print "ANGLE BETWEEN", math.fabs(path_angle - bike_angle)

		k = k + 1

		# When it crosses the line... ?


if __name__ == '__main__':
	
	new_bike = bikeState.Bike(-1, -15, 0.1, math.pi/3, 0, 0, 3.57)
	waypoints = [(0.1, 0.1), (30.1, 0.1), (31.1, 0.1)]
	new_map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
	new_nav = nav.Nav(new_map_model)
	# print new_nav.direction_to_turn()
	# print new_bike.rhs(new_nav.direction_to_turn())
	# PLOTTING
	plt.ion() # enables interactive plotting
	paths = new_map_model.paths
	fig = plt.figure()
	fig.set_dpi(100) #dots per inch
 	ax = plt.axes(xlim=(0, 20), ylim=(0, 20)) 
 	lc = mc.LineCollection(paths, linewidths=2, color = "black")
	ax.add_collection(lc)
	plt.show() 
	main_loop(new_nav, new_bike)
	