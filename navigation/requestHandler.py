import requests
import json
import urllib
import math

""" Server needs to be open """

#Gets url object and makes it into string
url = urllib.urlopen("http://localhost:8000/secretdatatransfer")
string = url.read()

#Make into dictionary
dictionary = json.loads(string)
legs = dictionary["legs"] #points list

#Gets latitude of first point
print legs[0]["points"][0]['lat']

def convert(longitude, latitude):
	""" Mercator map projection 
	http://stackoverflow.com/questions/14329691/covert-latitude-longitude-point-to-a-pixels-x-y-on-mercator-projection """

	mapWidth    = 1000;
	mapHeight   = 1000;

	x = (longitude+180)*(mapWidth/360.0)

	latRad = latitude*math.pi/180

	mercN = math.log(math.tan((math.pi/4)+(latRad/2)))

	y = (mapHeight/2)-(mapWidth*mercN/(2*math.pi))

	return (x,y)


print convert(-76.4852, 42.45026)
print convert(-76.48506, 42.450390000000006)

# LEG: A separate leg will be present for each waypoint or destination specified.
# Each leg consists of a series of steps

#latitude and longitude conversion to x, y coordinates
# dx = (lon2-lon1)*40000*math.cos((lat1+lat2)*math.pi/360)/360
# dy = (lat1-lat2)*40000/360



# int x =  (int) ((MAP_WIDTH/360.0) * (180 + lon));
# int y =  (int) ((MAP_HEIGHT/180.0) * (90 - lat));


#Print string
print len(legs)
