import numpy as np
import math
from constants import *


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

	
	def next_state(self, steerD):
		""" Equivalent to rhs in Matlab. Modifies the state object to turn it into the next state """

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
		self.xB = self.xB + xdot*TIMESTEP
		self.yB = self.yB + ydot*TIMESTEP
		self.phi = self.phi + phi_dot*TIMESTEP
		self.psi = self.psi+ psi_dot*TIMESTEP
		self.delta = self.delta + delta_dot*TIMESTEP
		self.w_r = self.w_r + wr_dot*TIMESTEP
		self.v = self.v + v_dot*TIMESTEP

		return u

