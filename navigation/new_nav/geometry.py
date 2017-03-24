
""" This module contains all the vector/math functions """

def unit_vector(p1, p2):
	""" Returns: the unit vector given points [p1] and [p2] """
	v =  np.array([p2[0]-p1[0], p2[1]-p1[1]])
	return v/np.linalg.norm(v)


def dist2(p1, p2):
	""" Returns: the square of the distance between [p1] and [p2] """
	return (np.square(point1[0]-point2[0]) + np.square(point1[1]-point2[1]))


def dist(p1, p2):
	""" Returns: the distance between points [p1] and [p2] """
	return np.sqrt(dist2(p1, p2))	


def nearest_point_on_path(path, point):
	""" Returns: the nearest point on the [path] to the [point] """
	point1 = path[0]
	point2 = path[1]
	dist = dist(point1, point2)	
	d2 = dist2(point1, point2)
	t = ((point[0] - point1[0])*(point2[0]-point1[0]) + (point[1] - point1[1]) * (point2[1] - point1[1]))/d2
	if (t < 0):
		return point1
	elif (t > 1):
		return point2
	else:
		x = point1[0] + t * (point2[0]-point1[0])
		y = point1[1] + t * (point2[1]-point1[1])
		return (x, y)


def get_sign(v):
	""" Returns: the sign of the number [v] """
	if v == 0:
		return 0
	else:
		return v/np.abs(v)


def angle_from_path(bike_direction, p1, p2):
	""" [angle_from_path] returns the angle that the bicycle has to turn to 
	be perpendicular to the line defined by [p1] and [p2] as well as if
	it has to turn clockwise or counterclockwise """
	bike_vector = (math.cos(bike_direction), math.sin(bike_direction))
	path_vector = (p2[0]-p1[0], p2[1]-p1[1])
	dot_product = bike_vector[0]*path_vector[0] + bike_vector[1]*path_vector[1]
	angle = get_sign(dot_product)*math.acos(dot_product/dist(point1, point2))
	return np.degrees(angle)


def distance_from_path(point, target_path):
	""" [distance_from_path] calculates the distance from [point] to [target_path] """
	# if target_path is None:
	# 	target_path = self.target_path
	p1 = np.array(target_path[0])
	v = unit_vector(target_path[0], target_path[1])
	if v[0] == 0:
		sign = get_sign(v[1])
		return  sign*(p1[0] - point[0])
	v_perp = np.array([v[1], -1*v[0]])
	bike_coords = np.array(point)
	r = p1 - bike_coords
	dist = np.sum(v_perp*r)
	return dist

