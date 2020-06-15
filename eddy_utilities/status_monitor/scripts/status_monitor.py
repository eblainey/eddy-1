#!/usr/bin/env python3

import sys

### ROS imports

import rospy
import rosnode
import rostopic
import rosservice

from std_msgs.msgs import Bool, ColorRGBA

class StatusMonitor:
	
	LED_DISABLED = (1.0, 0.0, 1.0)
	LED_WAITING = (0.0, 0.0, 1.0)
	LED_GOOD = (0.0, 1.0, 0.0)
	LED_WARNING = (1.0, 1.0, 0.0)
	LED_DANGER = (1.0, 0.5, 0.0)
	LED_FATAL = (1.0, 0.0, 0.0)
	
	ROSBAG_CONFIG = {
		"node" : "rosbag",
		"topic" : "/eddy/status/led/rosbag",
		"publisher" : None,
		"monitored_topics" : {
			"/rosbag/recording" : lambda x: return True
		},
		"monitored_services" : {
			"/rosbag/record/start" : None,
			"/rosbag/record/stop" : None 
		}
	}
	
	CONTROLLER_CONFIG = {
		"node" : "controller",
		"topic" : "/eddy/status/led/controller",
		"publisher" : None,
		"monitored_topics" : {
			"/eddy/joy" : lambda x: return True
		},
		"monitored_services" : {}
	}
	
	SONAR_CONFIG = {
		"node" : "sonar",
		"topic" : "/eddy/status/led/sonar",
		"publisher" : None,
		"monitored_topics" : {
			"/eddy/sonar/profile" : None,
			"/eddy/sonar/history" : None,
			"/eddy/sonar/distance" : None
		}
		"monitored_services" : {}
	}
	
	GPS_CONFIG = {
		"node" : "gps",
		"topic" : "/eddy/status/led/gps",
		"publisher" : None,
		"monitored_topics" : {
			"/eddy/gps/fix" : None,
			"/eddy/gps/vel" : None,
			"/eddy/gps/time_reference" : None
		},
		"monitored_services" : {}
	}
	
	CAMERA_CONFIG = {
		"node" : "camera",
		"topic" : "/eddy/status/led/camera",
		"publisher" : None,
		"monitored_topics" : {
			"/eddy/camera/camera_info" : None,
			"/eddy/camera/image_raw" : None
		},
		"monitored_services" : {}
	}
	
	def __init__(self, configs):
		
		for config in configs:
			config["publisher"] = rospy.Publisher(config["topic"], ColorRGBA, queue_size=1)
			pass
		
		self.rosbag_pub = rospy.Publisher("/eddy/status/led/rosbag", ColorRGBA, queue_size=1)
		self.controller_pub = rospy.Publisher("/eddy/status/led/controller", ColorRGBA, queue_size=1)
		self.sonar_pub = rospy.Publisher("/eddy/status/led/sonar", ColorRGBA, queue_size=1)
		self.gps_pub = rospy.Publisher("/eddy/status/led/gps", ColorRGBA, queue_size=1)
		self.camera_pub = rospy.Publisher("/eddy/status/led/camera", ColorRGBA, queue_size=1)
		pass
	
	def update(self):
		
		nodes = rosnode.get_node_names()
		pass

def main(args):
	# Initialize the node
	rospy.init_node('status_monitor', anonymous=False)
	
	static_args = [
		{ "id" :  },
		{  },
	]
	
	arg_defaults = {
		"config" : static_args
	}
	
	args = updateArgs(arg_defaults)
	
	monitor = StatusMonitor(**args)
	
	rospy.Timer(rospy.Duration(1), monitor.update)

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
