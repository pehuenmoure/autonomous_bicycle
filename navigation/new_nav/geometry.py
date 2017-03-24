
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


