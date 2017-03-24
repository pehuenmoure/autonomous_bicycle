
""" This module contains a class to represent the bike's state """

class Bike(object):

	def __init__(self, xB, yB, phi, psi, delta, w_r, v):
		""" State representation in Matlab """

		#xB, yB initial position coordinates of bicycle
		self.xB = xB
		self.yB = yB
		self.phi = phi #lean
		self.psi = psi #direction (aka thetaB)
		self.delta = delta # Steer Angle!!!!!!
		self.w_r = w_r 
		self.v = v  #speed
		self.turning_r = 2


	@property
	def vector(self):
		b = np.array([math.cos(self.psi), math.sin(self.psi)])
		return b/np.linalg.norm(b)