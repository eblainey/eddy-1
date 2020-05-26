#!/usr/bin/env python

### ROS imports

import rospy
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

### OpenCV imports

import cv2
import numpy as np

### BlueRobotics imports

from brping import Ping1D

class Sonar:

	MAX_RESOLUTION = 200
	MAX_HISTORY = 200

	def __init__(self, device):
		self.ping = Ping1D()
		
		if device is not None:
			self.ping.set_connect_serial(device)
		
		if self.ping.initialize() is False:
			rospy.logerror("Ping Sonar failed to initialize")
			exit(1) 
		
		data = self.ping.get_general_info()
		rospy.loginfo("Ping1D Info")
		rospy.loginfo("- Firmware version: %s.%s" % (data["firmware_version_major"],
			data["firmware_version_minor"]))
		rospy.loginfo("- Ping Interval: %s ms" % data["ping_interval"])
		rospy.loginfo("- Gain: %s - Mode: %s" % (data["gain_setting"], data["mode_auto"]))
		
		self.ping_rate = 1000 / int(data["ping_interval"])
		
		self.history_pub = rospy.Publisher('history', Image, queue_size=3)
		self.profile_pub = rospy.Publisher('profile', Float32MultiArray, queue_size=3)
		self.distance_pub = rospy.Publisher('distance', Range, queue_size=3)
		
		self.history = Sonar.MAX_HISTORY*[SONAR.MAX_RESOLUTION*[0.0]]
		
		self.bridge = CvBridge()
		pass
	
	def spin(self):
		rate = rospy.Rate(self.ping_rate)
		while not rospy.is_shutdown():
			time, data = rospy.Time.now(), self.ping.get_profile()
			
			self.publish_history_data(data, time)
			self.publish_profile_data(data, time)
			self.publish_distance_data(data, time)
			
			rate.sleep()
			pass
		pass
	
	def publish_history_data(self, time, data):
		
		self.history.append(data["profile_data"])
		self.history.pop(0)
		
		hue_image = np.array(self.history)
		np.transpose(hue_image)
		saturation_image = np.ones(MAX_RESOLUTION, MAX_HISTORY)
		value_image = np.ones(MAX_RESOLUTION, MAX_HISTORY)
		
		hsvimg = cv2.merge(hue_image, saturation_image, value_image)
		bgrimage = cv2.cvtColor(hsvimg, cv2.COLOR_HSV2BGR)
		
		try:
			self.history_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)
		pass
	
	def publish_profile_data(self, time, data):
		self.profile_pub.publish(Float32MultiArray(data=data["profile_data"]))
		pass
	
	def publish_distance_data(self, time, data):
		range_msg = Range()
		
		range_msg.header.stamp = time
		range_msg.header.frame_id = "sonar"
		range_msg.header.seq = data["ping_number"]
		
		range_msg.radiation_type = Range.ULTRASOUND
		range_msg.min_range = data["scan_start"] / 1000.0
		range_msg.max_range = (data["scan_start"] + data["scan_length"]) / 1000.0
		
		range_msg.range = data["distance"] / 1000.0
		
		self.distance_pub.publish(range_msg)
		pass

def main(args):
	# Initialize the node
	rospy.init_node('sonar', anonymous=False)

	arg_defaults = {
		"device" : "/dev/ttyUSB0"
	}
	args = updateArgs(arg_defaults)
	
	sonar = Sonar(**args)
	
	try:
		sonar.spin()
	except rospy.ROSInterruptException, e:
		print e

def updateArgs(arg_defaults):
	# Look up parameters starting in the node's private parameter space, but also search outer namespaces.
	args = {}
	for name, val in arg_defaults.iteritems():
		full_name = rospy.search_param(name)
		print "name ", name, full_name
		if full_name is None:
			args[name] = val
		else:
			args[name] = rospy.get_param(full_name, val)
			print "We have args " + val + " value " + args[name]
	return (args)

if __name__ == '__main__':
	main(sys.argv)
