import numpy as np
import math
from constants import *
import nav

class MainNavigation(object):

	# We could also call mainNavigation giving it a list of waypoints instead of list of x coords and list of y coords
	def __init__(self, x, y):
		"""INSTANCE ATTRIBUTES: 
		x [float list]: x - coordinates of waypoints
		y [float list]: y - coordinates of waypoints """
		
		self.x_coords = x
		self.y_coords = y
		self.tstep = TIMESTEP 
		self.tarray = [] 
		self.currentSegment = 0
		self.xC = x[self.currentSegment]
		self.yC = y[self.currentSegment]
		self.xD = x[self.currentSegment+1] 
		self.yD = y[self.currentSegment+1]
		self.state = self.initialize_bike_state()
		self.all_states = [] # list of state objects to keep track of all different states
		self.waypoints = zip(x, y)
		self.nav_commands = []
		self.mot_commands = []

	
	def initialize_bike_state(self):
		""" """
		xB = -1.01
	   	yB = -1.01 #xB, yB initial position coordinates of bicycle
	   	phi0 = 0.1 #lean
	   	yaw0 = math.pi/3 #heading 
	   	delta0 = 0 #steer angle
	   	phiDot0 = 0 #lean rate
	   	v = 3.57 #velocity
	   	return State(xB,yB,phi0,yaw0,delta0,phiDot0,v)

	
	def loop(self):
		""" Main loop that calls nav algorithm and updates state at each step """

		k = 0
		self.all_states.append(self.state) 

		while (k < 10):
				
			xB = self.state.xB
			yB = self.state.yB
			phiB = self.state.phi

			#if the lean angle is too high, the test should count as a failure
			if (abs(phiB) >= math.pi/4):
				print "Bike has fallen; test failure"
				exit()

			thetaB = self.state.psi
			v = self.state.v

			self.tarray.append(k*TIMESTEP)

			steerD = nav.command(xB, yB, thetaB, v, self.waypoints)
			steerD = steerD * MAX_STEER

			self.nav_commands.append(steerD)

			# Call rhs
			rhs_result = self.state.rhs(steerD)
			u = rhs_result[0]
			values = rhs_result[1]

			# Gathers motor commands
			self.mot_commands.append(u)

			# Values of new state
			new_xB = self.state.xB + values[0]*TIMESTEP
			new_yB = self.state.yB + values[1]*TIMESTEP
			new_phi = self.state.phi + values[2]*TIMESTEP
			new_psi = self.state.psi+ values[3]*TIMESTEP
			new_delta = self.state.delta + values[4]*TIMESTEP
			new_w_r = self.state.w_r + values[5]*TIMESTEP
			new_v = self.state.v + values[6]*TIMESTEP

			self.state = State(new_xB, new_yB, new_phi, new_psi, new_delta, new_w_r, new_v)
			self.all_states.append(self.state) 
			
			k = k+1

		return self.all_states

    	# # Return list of all state objects gathered
    	# return self.all_states


class State(object):

	def __init__(self, xB, yB, phi, psi, delta, w_r, v):
		""" State representation in Matlab """

		#xB, yB initial position coordinates of bicycle
		self.xB = xB
		self.yB = yB
		self.phi = phi #lean
		self.psi = psi
		self.delta = delta # Steer Angle!!!!!!
		self.w_r = w_r
		self.v = v

	
	def rhs(self, steerD):
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

		# Returns u which is the motor command and the zdot vector in the form of a list
		return (u, [xdot, ydot, phi_dot, psi_dot, delta_dot, wr_dot, v_dot])

if __name__ == '__main__':

	mainNav = MainNavigation([0.1, 30.1, 31.1], [0.1,0.1,0.1])

	print mainNav.loop()



