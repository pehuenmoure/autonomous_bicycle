import nav
import mapModel
import math
import bikeState

if __name__ == '__main__':
	new_bike = bikeState.Bike(0, -10, 0.1, math.pi/3, 0, 0, 3.57)
	waypoints = [(0.1, 0.1), (30.1, 0.1), (31.1, 0.1)]
	new_map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
	new_nav = nav.Nav(new_map_model)
	