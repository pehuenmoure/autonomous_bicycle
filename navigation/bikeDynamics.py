import numpy as np
import math
from constants import *

class MainNavigation(object):


	def __init__(self, x, y):
		"""INSTANCE ATTRIBUTES: 
		x [float list]: x - coordinates of waypoints
		y [float list]: y - coordinates of waypoints """
		
		self.x_coords = x
		self.y_coords = y
		self.tstep = TIMESTEP 
		self.tarray = []
		self.currentSegment = 1



class State(object):

	def __init__(self, xB, yB, phi, psi, delta, w_r, v):
		""" State representation in Matlab """

		self.xB = xB
		self.yB = yB
		self.phi = phi
		self.psi = psi
		self.delta = delta
		self.w_r = w_r
		self.v = v

	
	def rhs(self, steerD):
		""" Modifies the state object to turn it into the next state """
		deltaD = steerD
		u = K1 * self.phi + K2 * self.w_r + K3 * (self.delta - deltaD)

		if u>10:
			u = 10
		else:
			u = -10

		xdot = self.v * np.cos(self.psi)
		ydot = self.v * np.sin(self.psi)
		phi_dot = self.w_r
		psi_dot = (self.v/L)*(np.tan(self.delta)/np.cos(self.phi))
		delta_dot = u # ideal steer rate
		v_dot = 0
		wr_dot = (((-(self.v)**2)*self.delta) - B * self.v * u + G * L * self.phi)/(H*L)

		# modifying state object 
		self.xB = xdot
		self.yB = ydot
		self.phi = phi_dot
		self.psi = psi_dot
		self.delta = delta_dot
		self.w_r = wr_dot
		self.v = v_dot

		return u

