# constants.py
import math

######### mainNavigation constants

# properties of the p struct in matlab
G = 9.81
L = 1.02
B = 0.3
H = 0.9
# trail is zero
C = 0 

# gains
K1 = 71.
K2 = 21.
K3 = -20.

# time
TIMESTEP = 1.0/100
PAUSE = TIMESTEP

# used in converting output of nav to steerD
MAX_STEER = math.pi/6.0