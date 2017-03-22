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
		""" Main loop that calls nav algorithm and then accounts for bike 
		dynamics and updates state at each step """

		# plt.plot(self.state.xB, self.state.yB, 'ro')
		# plt.show()
	
		k = 0
		self.all_states.append(self.state) 

		# hl, = plt.plot([], [])
		# plt.show()

		while (k < 800):
			# if k > 200:
			# 	print "OVERHSHOOT", self.overshoot_distance()
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

			# slope = (new_yB - self.state.yB)/(new_xB - self.state.xB)
			# print "SLOPE IS", slope

			self.state = State(new_xB, new_yB, new_phi, new_psi, new_delta, new_w_r, new_v)
			self.all_states.append(self.state) 
			
			k = k+1

		return self.all_states

    	# # Return list of all state objects gathered
    	# return self.all_states

    #############################################################################
   	# IDEA
	# find next zero slope point in x-coord vs y-coord graph (bike trajecory)
	# calculate distance from line to point (overshoot)
	# increase delta by that distance in nav

	############################################################################

	""" Hey guys! So I want to explain what this code is in here. 

	Basically, the function called loop is the main loop that was in matlab which 
	calls the navigation algorithm and updates the state. It also calls a function called 
	rhs (See below in class State) which has couple of equations. I accumulate a list of
	all states that passed with the idea that we could either create plots after all states 
	have been calculated (I am guessing that will be quicker) or we can do it like I have it now,
	dynamically updating the plot during the while loop (Which is more fun :D). 

	What I worked on today was figuring out a way to correct the overshoot using the iterative
	approach we talked about. So my idea was to have a function that finds the point where
	the slope is zero (yes this is very simplistic that works for a horizontal path) and then 
	find the distance between that point and the target path and make it be the overshoot.
	Then we could add that distance to delta to correct it. Basically, "max_overshoot_point" 
	creates a copy of the currrent state and runs a loop until it finds the point where slope is 
	zero. I also created some functions in nav to allow me to just use some functions we have 
	already written instead of rewriting them, spacifically the functions are target_path_idx and
	distance_from_target_path. All this is to calculate the overshoot. 

	This is definitely not a good way. Ultimately instead of finding the zero slope point
	it would be best to determine that point from the variables of the bike's state. Like 
	find a way to know when countersteering starts and when it ends from the lean or the steer etc. 
	But I have no idea what that condition is so I am trying to do it with the bike trajectory graph.

	So I do have the overshoot function BUT I don't call it from nav now... Because I 
	would have to import bikeDynamics in nav and I am trying to avoid it as long as I can because that 
	might get messy and we might have to pass the bikeDynamics state to nav so that it can then call 
	bikeDynamics.py and know the state.

	What I tried to test this whole theory was calling the function to calculate the
	overshoot in the loop and then manually hardcoded the overshoot distance in nav and it 
	improved the result so much! I will post a picture in slack! (Still doesn't get in a straight line 
	thought but oscillations are a lot better)

	To calculate overhsoot distance uncomment lines 81 and 82 in the code to see how slot it is. I have k > 200 
	because there are other zero slope points in the graph and I only want the one of the first oscillation.

	THIS IS SO SLOW though :'( :'( 

	"""

	def max_overshoot_point_state(self):
		""" Finds the state at the point where slope becomes zero """

		same_rate = True # To figure out when we pass the slope = 0 point

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
				if ((slopes[len(slopes)-1] > 0 and slope < 0) or (slopes[len(slopes)-1]  < 0 and slope > 0)):
					same_rate = False

			slopes.append(slope)

			current_state = State(new_xB, new_yB, new_phi, new_psi, new_delta, new_w_r, new_v)

		return current_state


	def overshoot_distance(self):
		""" Calculates overshoot distance """
		state = self.max_overshoot_point_state()
		target_path = nav.target_path_idx(self.state.xB, self.state.yB, self.state.psi, self.state.v, self.waypoints)
		overshoot = nav.distance_from_target_path(state.xB, state.yB, state.psi, state.v, self.waypoints, target_path)
		# print "X COORDINATE", state.xB
		# print "Y COORDINATE", state.yB
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



