"""
Matplotlib Animation Example

author: Jake Vanderplas
email: vanderplas@astro.washington.edu
website: http://jakevdp.github.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib import collections  as mc
from pprint import pprint
import struct
import time
import nav

"""
Built of polygon class it allows for rotation 
"""
class bike_sim (plt.Polygon):
	def __init__ (self, bike):
		self.bike = bike
		self.x = bike.xy_coord[0]
		self.y = bike.xy_coord[1]
		self.h = bike.h
		self.width = .15
		self.theta = bike.direction
		plt.Polygon.__init__(self, self.get_coordinates(self.x, self.y, self.theta))

	def get_coordinates(self, x, y, theta):
		x1 = x + self.width * np.sin(theta) 
		y1 = y - self.width * np.cos(theta)
		x2 = x - self.width * np.sin(theta) 
		y2 = y + self.width * np.cos(theta)
		x3 = x + self.h * np.cos(theta) 
		y3 = y + self.h * np.sin(theta)
		return [[x1,y1], [x2,y2], [x3,y3]]

	def move_straight(self):
		v = self.bike.speed
		x = self.x + v * np.cos(self.theta) 
		y = self.y + v * np.sin(self.theta)
		self.set_xy(self.get_coordinates(x, y, self.theta))
		self.x = x
		self.y = y
		return self

	'''
	Positive phi turns the bike counter clockwise
	Negative phi turns the bike clockwise
	'''
	def rotate(self, sign, radius):
		v = self.bike.speed
		phi = sign*v/float(radius)
		theta = np.mod(self.theta - sign*np.pi/2, 2*np.pi)
		theta2 = np.mod(theta + phi, 2*np.pi)
		x2 = self.x + radius * (np.cos(theta2) - np.cos(theta))
		y2 = self.y + radius * (np.sin(theta2) - np.sin(theta))
		theta2 = np.mod(theta2 + sign*np.pi/2, 2*np.pi)
		self.set_xy(self.get_coordinates(x2, y2, theta2))
		self.theta = theta2
		self.x = x2
		self.y = y2
		return self

	def update_bike(self):
		self.bike.xy_coord = (self.x, self.y)
		self.bike.direction = self.theta


class Simulator(object):

	def __init__(self, map_model, nav):
		""" Nav initializer """
		self.map_model = map_model
		self.nav = nav
		self.fig = plt.figure()
		self.fig.set_dpi(100)
		ax = plt.axes(xlim=(0, 20), ylim=(0, 20))
		self.bike_sim = bike_sim(map_model.bike)
		ax.add_patch(self.bike_sim)
		lc = mc.LineCollection(self.map_model.paths, linewidths=2,
			color = "black")
		ax.add_collection(lc)
		plt.plot(self.map_model.waypoints[0], self.map_model.waypoints[1], 'ro')

	def run(self):
		anim = animation.FuncAnimation(self.fig, self.update_bike,  
                                frames=360, 
                                interval=10)
		plt.show()

	def update_bike(self, i):
		self.bike_sim.update_bike()
		dir_to_turn = self.nav.direction_to_turn()
		if dir_to_turn == 0:
			#print('straight')
			self.bike_sim.move_straight()
		else:
			#print 'ccw' if dir_to_turn == 1 else 'cw'
			self.bike_sim.rotate(dir_to_turn, self.bike_sim.bike.turning_r)
		return self.bike_sim,


