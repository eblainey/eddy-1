#!/usr/bin/python3
from bokeh.plotting import figure, show, ColumnDataSource, curdoc
from bokeh.models import Triangle
from bokeh.tile_providers import get_provider, Vendors
import numpy as np
from pyproj import Proj, transform
import signal
import rospy
from sensor_msgs.msg import NavSatFix
import sys
import math
from bokeh.events import DoubleTap
"""
    Goal: Show live GPS data on html without java.
    >MileStone 1- Convert long/lat to mercator data -DONE
    >MileStone 2- Start with one GPS data and build map around it -DONE
    >MileStone 3- Generate random data and update in html page -DONE
    >MileStone 4- Add line to show path -DONE
    >MileStone 5- Integrate ROS -DONE
    >MileStone 6- Object oriented
    To kill the server:
        netstat -tupln
        kill -9 <pid>
"""
# Setting pyproj
outProj = Proj(init='epsg:3857')
inProj = Proj(init='epsg:4326')

class map:
    def __init__(self):
        self.rollover = 1               # number of data to be shown
        self.rolloverline = 10000000000 # number of data to be shown
        subscriber = rospy.Subscriber("/eddy/gps/fix", NavSatFix, self.readGPS, queue_size=1)
        self.clean_initData = False

        # Starting GPS position
        self.lonGPS = -59.64096083333333
        self.latGPS = 13.191748333333333
        # Convert to epsg:3857 COORDINATE
        self.x, self.y = transform(inProj,outProj,self.lonGPS,self.latGPS)
        print ("Starting Coordinates: ", self.x, self.y)

        # DEFINE COORDINATE RANGE IN WEB MERCATOR COORDINATE SYSTEM (EPSG:3857)
        # Map width and height setting
        self.width = 10
        self.height = 100
        self.x_range, self.y_range=([self.x + self.width, self.x - self.width],
        [self.y - self.height, self.y + self.height])

        #BOKEH COLUMN DATA SOURCE
        self.source = ColumnDataSource(data=dict(longitude=[], latitude=[]))
        self.sourceLine = ColumnDataSource(dict(x=[], y=[]))
        self.sourceTarget = ColumnDataSource(data=dict(x=[], y=[]))
        self.coordList=[]

        # Heading calculation
        self.angle = 0 # Heading angle of the robot
        self.old_lat = 0
        self.old_lon = 0

        #SETUP FIGURE
        self.p = figure(x_range=self.x_range, y_range=self.y_range, x_axis_type='mercator', y_axis_type='mercator',
                        sizing_mode='scale_width', plot_height=200)
        self.p.add_tile(get_provider(Vendors.CARTODBPOSITRON))
        self.p.triangle(x='longitude', y='latitude', source=self.source, fill_color="red",
                        size=10,fill_alpha=0.8,line_width=0.5, angle=self.angle)
        self.p.line("x", "y", source=self.sourceLine, line_width = 2)
        self.p.circle_cross("x", "y", source=self.sourceTarget,fill_color="yellow", size=10)
        self.p.on_event(DoubleTap, self.getPoints)

    def readGPS(self, data):
        self.lonGPS = data.longitude
        self.latGPS = data.latitude
        if (self.old_lat !=0 and self.old_lat!=0):
            self.angle = self.get_bearing(self.old_lat,self.old_lon, self.latGPS, self.lonGPS)
        self.old_lat = self.latGPS
        self.old_lon = self.lonGPS
        if self.clean_initData == False:
            # This is to clean the line before starting
            for x in range(len(self.sourceLine.data["x"])):
                self.sourceLine.data["x"].pop(-1)
                self.sourceLine.data["y"].pop(-1)
            self.clean_initData = True

    def getPoints(self, event):
        # GPS target point for waypoint navigation
        # NOT COMPLETE
        Coords=(event.x, event.y)
        self.coordList.append(Coords)
        self.sourceTarget.data = dict(x=[i[0] for i in self.coordList], y=[i[1] for i in self.coordList])

    def get_bearing(self, lat1, lon1, lat2, lon2):
        dLon = lon2 - lon1
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
        brng = np.rad2deg(math.atan2(y, x))
        if brng < 0: brng+= 360
        math.radians(brng)
        return brng

    def update(self):
        x, y = transform(inProj,outProj,self.lonGPS,self.latGPS)
        self.source.stream(dict(longitude=[x], latitude=[y]), rollover=self.rollover)
        if self.clean_initData == True:
            self.sourceLine.stream(dict(x=[x], y=[y]), rollover=self.rolloverline)

    def signal_handler(self, sig, frame):
        print('You pressed Ctrl+C!')
        sys.exit()


rospy.init_node('GPS', anonymous=False)
UPDATE_INTERVAL = 1000 # Refresh rate in milliseconds
mapping = map()
signal.signal(signal.SIGINT, mapping.signal_handler) # This is for handling Ctrl C
doc = curdoc()
doc.add_root(mapping.p)
doc.add_periodic_callback(mapping.update, UPDATE_INTERVAL)
