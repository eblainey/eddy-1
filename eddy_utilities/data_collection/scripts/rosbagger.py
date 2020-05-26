#!/usr/bin/env python3

import rospy, rosbag

from std_srvs.srv import *
from std_msgs.msg import Bool

import subprocess
import signal
import os

from os import path
from time import time
from datetime import datetime

class ROSBagger:
	def __init__(self, bag_name, data_directory, topics, start_recording):
		self.bag_name = bag_name
		self.rosbag_started = False
		self.rosbag_process = None
		self.data_directory = data_directory
		self.subscribed_topics = topics

		# Initialize the recording service
		recorder_service_start = rospy.Service('rosbag/record/start', Empty, self.start_rosbag)
		recorder_service_stop = rospy.Service('rosbag/record/stop', Empty, self.stop_rosbag)
		self.status_pub = rospy.Publisher('rosbag/recording', Bool, latch=True, queue_size=3)
		
		self.status_pub.publish(Bool(False))
		
		if start_recording:
			self.start_rosbag(EmptyRequest())
			rospy.spin()
		pass

	def start_rosbag(self, request):
		if self.rosbag_started:
			return EmptyResponse()
		
		new_args = updateArgs({ 'topics' : [] })
		self.subscribed_topics = new_args['topics']
		rospy.loginfo(self.subscribed_topics)
		
		# Start recording new rosbag
		timestamp = datetime.fromtimestamp(time()).strftime('%Y-%m-%d-%H-%M-%S')
		file_name = path.join(self.data_directory, self.bag_name + '-'+timestamp+'.bag')
		args = ["rosbag", "record", "-O", "__name:=eddy_recorder", file_name] + self.subscribed_topics
		
		subprocess.Popen(args)
		self.rosbag_started = True
		
		self.status_pub.publish(Bool(True))
		
		rospy.loginfo('Starting to record rosbag ...')
		
		return EmptyResponse()

	def stop_rosbag(self, request):
		if not self.rosbag_started:
			return EmptyResponse()
		
		# Stop recording rosbag
		self.rosbag_started = False
		subprocess.call(["rosnode", "kill", "eddy_recorder"])
		self.status_pub.publish(Bool(False))
		
		rospy.loginfo('Stopped recording rosbag ...')
		
		return EmptyResponse()

def main(args):
	# Initialize the node
	rospy.init_node('rosbagger', anonymous=False)

	arg_defaults = {
		'bag_name' : "test",
		'topics' : [
			"-a"
		],
		'data_directory' : '/home/Desktop',
		'start_recording' : False
	}
	args = updateArgs(arg_defaults)

	ROSBagger(**args)
	
	try:
		rospy.spin()
	except rospy.ROSInterruptException as e:
		rospy.logerror(e)

def updateArgs(arg_defaults):
	# Look up parameters starting in the node's private parameter space, but also search outer namespaces.
	args = {}
	for name, val in arg_defaults.items():
		full_name = rospy.search_param(name)
		rospy.loginfo("name %s, %s" % (name, full_name))
		if full_name is None:
			args[name] = val
		else:
			args[name] = rospy.get_param(full_name, val)
			rospy.loginfo("We have args %s value %s" % (val, args[name]))
	return (args)

if __name__ == '__main__':
	main(sys.argv)
