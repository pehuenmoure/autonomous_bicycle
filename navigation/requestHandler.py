import requests
import json
import urllib
import math

""" Run server """

def convert(latitude, longitude):
	""" Mercator map projection 
	http://stackoverflow.com/questions/14329691/covert-latitude-longitude-point-to-a-pixels-x-y-on-mercator-projection """

	mapWidth    = 20
	mapHeight   = 20

	x = (longitude+180)*(mapWidth/360.0)

	latRad = latitude*math.pi/180

	mercN = math.log(math.tan((math.pi/4)+(latRad/2)))

	y = (mapHeight/2)-(mapWidth*mercN/(2*math.pi))

	return (x,y)

def convert2(latitude, longitude):
	"""http://stackoverflow.com/questions/1019997/convert-lat-longs-to-x-y-co-ordinates"""
	mapWidth    = 1000
	mapHeight   = 1000

	y = ((-1 * latitude) + 90) * (mapHeight / 180.0);
	x = (longitude + 180) * (mapWidth / 360.0);

	return (x, y)
	
# Also this --> http://stackoverflow.com/questions/1019997/convert-lat-longs-to-x-y-co-ordinates

def parse_json():
	#Gets url object and makes it into string
	url = urllib.urlopen("http://localhost:8000/secretdatatransfer")
	string = url.read()

	#Make into dictionary
	dictionary = json.loads(string)
	legs = dictionary["legs"] #points list

	points = []
	points2 = []

	#Accumulates list of point tuples
	for i in legs:
		for j in (i["points"]):
			latitude = j['lat']
			longitude = j['lng']
			xy = convert(latitude, longitude)
			xy2 = convert2(latitude, longitude)
			points.append(xy)
			points2.append(xy2)

	# Scaling Issue of points. Need to ZOOM IN because discrepancies are very small
	print points
	print points2
	return points


parse_json()

#Gets latitude of first point
# print legs[0]["points"][0]['lat']



# print convert(-76.4852, 42.45026)
# print convert(-76.48506, 42.450390000000006)

# LEG: A separate leg will be present for each waypoint or destination specified.
# Each leg consists of a series of steps

#latitude and longitude conversion to x, y coordinates
# dx = (lon2-lon1)*40000*math.cos((lat1+lat2)*math.pi/360)/360
# dy = (lat1-lat2)*40000/360



# int x =  (int) ((MAP_WIDTH/360.0) * (180 + lon));
# int y =  (int) ((MAP_HEIGHT/180.0) * (90 - lat));


#Print string
# print len(legs)
