import numpy as np
import math
from constants import *
import nav
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib import collections  as mc

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

		# PLOTTING
		plt.ion() # enables interactive plotting
		self.paths = self.initialize_paths(self.waypoints)
		self.fig = plt.figure()
		self.fig.set_dpi(100) #dots per inch
 		ax = plt.axes(xlim=(0, 20), ylim=(0, 20)) 
 		lc = mc.LineCollection(self.paths, linewidths=2, color = "black")
		ax.add_collection(lc)
		plt.show() 


	def initialize_paths(self, waypoints):
		""" returns list of paths from waypoints """
		paths = []
		if len(waypoints) < 2:
			return paths
		else:
			for i in range(1, len(waypoints)):
				paths.append((waypoints[i-1], waypoints[i]))
			return paths


	def initialize_bike_state(self):
		""" """
		xB = -1
	   	yB = -5 #xB, yB initial position coordinates of bicycle
	   	phi0 = 0.1 #lean
	   	yaw0 = math.pi/3 #heading 
	   	delta0 = 0 #steer angle
	   	phiDot0 = 0 #lean rate
	   	v = 3.57 #velocity
	   	return State(xB,yB,phi0,yaw0,delta0,phiDot0,v)

	
	def loop(self):
		""" Main loop that calls nav algorithm and updates state at each step """
		# plt.plot(self.state.xB, self.state.yB, 'ro')
		# plt.show()
	
		k = 0
		self.all_states.append(self.state) 

		# hl, = plt.plot([], [])
		# plt.show()

		while (k < 1000):

			xB = self.state.xB
			yB = self.state.yB
			phiB = self.state.phi

			# PLOT
			#Bike trajectory
			plt.scatter(xB, yB)
			plt.show()
			plt.pause(0.0000001)

			# Lean vs Time
			# plt.scatter((k+1)*TIMESTEP, self.state.delta)
			# plt.show()
			# plt.pause(0.0001)

			# Steer vs Time
			# plt.scatter((k+1)*TIMESTEP, self.state.delta)
			# plt.show()
			# plt.pause(0.00001)

			#if the lean angle is too high, the test should count as a failure
			if (abs(phiB) >= math.pi/4.0):
				print "Bike has fallen; test failure"
				exit()

			thetaB = self.state.psi
			v = self.state.v

			self.tarray.append(k*TIMESTEP)

			steerD = nav.command(xB, yB, thetaB, v, self.waypoints)
			print steerD

			steerD = steerD * MAX_STEER * (-1)

			print "Steer", self.state.delta
			print "NAV STEER COMMAND", steerD

			self.nav_commands.append(steerD)

			# Call rhs
			rhs_result = self.state.rhs(steerD)
			u = rhs_result[0]
			values = rhs_result[1]

			# Gathers motor commands
			self.mot_commands.append(u)
			print "Mot command is (steer rate) ", u

			# plt.scatter(TIMESTEP*k, u)
			# plt.show()
			# plt.pause(0.00001)

			# Values of new state
			new_xB = self.state.xB + values[0]*TIMESTEP
			new_yB = self.state.yB + values[1]*TIMESTEP
			new_phi = self.state.phi + values[2]*TIMESTEP
			new_psi = self.state.psi+ values[3]*TIMESTEP
			new_delta = self.state.delta + values[4]*TIMESTEP
			new_w_r = self.state.w_r + values[5]*TIMESTEP
			new_v = self.state.v + values[6]*TIMESTEP


			# print "SLOPE IS", slope

			self.state = State(new_xB, new_yB, new_phi, new_psi, new_delta, new_w_r, new_v)
			self.all_states.append(self.state) 
			
			k = k+1

		return self.all_states

    	# # Return list of all state objects gathered
    	# return self.all_states


   	# IDEA
	# find next zero slope point 
	# calculate distance form line to point
	# increase delta by that distance

	def max_overshoot_point_state(self):
		""" Finds point where slope bcomes zero """
		same_rate = true
		#create copy of current state
		current_state = State(self.state.xB, self.state.yB, self.state.phi, self.state.psi, self.state.delta, self.state.w_r, self.state.v)

		slopes = []


		while (same_rate):

			xB = current_state.xB
			yB = current_state.yB
			phiB = current_state.phi


			thetaB = current_state.psi
			v = current_state.v

			steerD = nav.command(xB, yB, thetaB, v, self.waypoints)

			steerD = steerD * MAX_STEER * (-1)

			# Call rhs
			rhs_result = current_state.rhs(steerD)
			u = rhs_result[0]
			values = rhs_result[1]

			# Values of new state
			new_xB = current_state.xB + values[0]*TIMESTEP
			new_yB = current_state.yB + values[1]*TIMESTEP
			new_phi = current_state.phi + values[2]*TIMESTEP
			new_psi = current_state.psi+ values[3]*TIMESTEP
			new_delta = current_state.delta + values[4]*TIMESTEP
			new_w_r = current_state.w_r + values[5]*TIMESTEP
			new_v = current_state.v + values[6]*TIMESTEP

			slope = (new_yB - current_state.yB)/(new_xB - current_state.xB)

			if (len(slopes) != 0):
				if ((slopes[len(slopes-1)] > 0 and slope < 0) or (slopes[len(slopes-1)]  < 0 and slope > 0)):
					same_rate = false

			slopes.append(slope)

			current_state = State(new_xB, new_yB, new_phi, new_psi, new_delta, new_w_r, new_v)

		return current_state

	def overshoot_distance(self):
		""" Calculates overshoot distance """
		state = self.max_overshoot_point_state()
		target_path = nav.command(self.state.xB, self.state.yB, self.state.psi, self.state.v, self.waypoints)
		overshoot = nav.distance_from_target_path(state.xB, state.yB, state.psi, state.v, self.waypoints, target_path)
		return overshoot


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

		if u > 10:
			u = 10
		elif u < -10:
			u = -10

		xdot = self.v * np.cos(self.psi)
		ydot = self.v * np.sin(self.psi)
		phi_dot = self.w_r
		psi_dot = (self.v/L)*(np.tan(self.delta)/np.cos(self.phi))
		delta_dot = u # ideal steer rate
		v_dot = 0
		# wr_dot = (((-(self.v)**2)*self.delta) - B * self.v * u + G * L * self.phi)/(H*L)
		wr_dot = (1/H)*(G*np.sin(self.phi) - np.tan(self.delta)*((self.v**2)/L + B*v_dot/L + np.tan(self.phi)*((B/L)*self.v*phi_dot - (H/(L**2)*(self.v**2)*np.tan(self.delta))))-B*self.v*delta_dot/(L*np.cos(self.delta)**2))
		# Returns u which is the motor command and the zdot vector in the form of a list
		return (u, [xdot, ydot, phi_dot, psi_dot, delta_dot, wr_dot, v_dot])

# class Plot(object):

# 	def __init__(self):

# 		self.fig = plt.figure()
# 		self.fig.set_dpi(100) #dots per inch
# 		ax = plt.axes(xlim=(0, 50), ylim=(0, 50)) 

# 		ax.add_patch(self.bike_sim)
# 		lc = mc.LineCollection(self.map_model.paths, linewidths=2,
# 			color = "black")
# 		ax.add_collection(lc)
# 		plt.plot(self.map_model.waypoints[0], self.map_model.waypoints[1], 'ro')



if __name__ == '__main__':

	mainNav = MainNavigation([0.1, 30.1, 31.1], [0.1,0.1,0.1])

	print mainNav.loop()



