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
matplotlib.use('TKAgg');
from matplotlib import pyplot as plt
from matplotlib import animation
from pprint import pprint

# First set up the figure, the axis, and the plot element we want to animate
fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 6.5)

ax = plt.axes(xlim=(0, 10), ylim=(0, 10))
bike = plt.arrow( 5, 8, 0.0, -0.2)
ax.add_patch(bike)

points = [(2,3), (4,8), (6,8), (8,7)]

points = ax.plot([2, 4, 6, 8], [3, 8, 4, 7], 'ro')


# initialization function: plot the background of each frame
def init():
	#ax.add_patch(circle)
	return bike,

# animation function.  This is called sequentially
def animate(i):
    pprint(vars(bike))
    #
    return bike,

# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=360, 
                               interval=5)

# save the animation as an mp4.  This requires ffmpeg or mencoder to be
# installed.  The extra_args ensure that the x264 codec is used, so that
# the video can be embedded in html5.  You may need to adjust this for
# your system: for more information, see
# http://matplotlib.sourceforge.net/api/animation_api.html
#anim.save('simulator.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

plt.show()