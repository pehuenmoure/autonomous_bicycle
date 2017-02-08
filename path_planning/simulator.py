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
from pprint import pprint
import struct

"""
Built of polygon class it allows for rotation 
"""
class triangle (plt.Polygon):
	def __init__ (self, x, y, h, r, theta):
		self.x = x
		self.y = y
		self.h = h
		self.r = r
		self.theta = theta
		plt.Polygon.__init__(self, self.get_coordinates(x, y, h, r, theta))

	def get_coordinates(self, x, y, h, r, theta):
		x1 = x + r * np.sin(np.radians(theta)) 
		y1 = y - r * np.cos(np.radians(theta))
		x2 = x - r * np.sin(np.radians(theta)) 
		y2 = y + r * np.cos(np.radians(theta))
		x3 = x + h * np.cos(np.radians(theta)) 
		y3 = y + h * np.sin(np.radians(theta))
		return [[x1,y1], [x2,y2], [x3,y3]]

	def move_straight(self, v):
		x = self.x + v * np.cos(np.radians(self.theta)) 
		y = self.y + v * np.sin(np.radians(self.theta))
		self.set_xy(self.get_coordinates(x, y, self.h, self.r, self.theta))
		self.x = x
		self.y = y
		return self

	def rotate(self, phi, radius):
		temp_t = np.mod(self.theta + phi, 360)
		temp_x = self.x + radius * np.cos(np.radians(temp_t))
		temp_y = self.y + radius * np.sin(np.radians(temp_t))
		self.set_xy(self.get_coordinates(temp_x, temp_y, self.h, self.r, temp_t))
		self.theta = temp_t
		self.x = temp_x
		self.y = temp_y
		return self


# First set up the figure, the axis, and the plot element we want to animate
fig = plt.figure()
fig.set_dpi(100)

ax = plt.axes(xlim=(0, 10), ylim=(0, 10))
bike = triangle(5, 8, 0.5, 0.15, 0)

ax.add_patch(bike)
pprint((bike.x, bike.y, bike.theta))

points = [(2,3), (4,8), (6,8), (8,7)]

points = ax.plot([2, 4, 6, 8], [3, 8, 4, 7], 'ro')


# animation function.  This is called sequentially
def update_bike(i):
	bike.rotate(-1, 0.01)
	pprint((bike.x, bike.y, bike.theta))
	return bike,


# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, update_bike,  
                               frames=360, 
                               interval=20)

# save the animation as an mp4.  This requires ffmpeg or mencoder to be
# installed.  The extra_args ensure that the x264 codec is used, so that
# the video can be embedded in html5.  You may need to adjust this for
# your system: for more information, see
# http://matplotlib.sourceforge.net/api/animation_api.html
#anim.save('simulator.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

plt.show()