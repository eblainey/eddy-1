#!/usr/bin/env python3

import sys

### ROS imports

import rospy
from std_srv.srvs import Empty
from std_msg.msgs import Bool
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry


def Controller:

	THRUST_AXIS = 0
	STEER_AXIS = 1

	ARMED_BUTTON = 0
	RECORD_BUTTON = 3

	def __init__(self, **params):
		
		self.max_forward_vel = params["max_forward_vel"]
		self.max_reverse_vel = params["max_reverse_vel"]
		self.max_rotational_vel = params["max_rotational_vel"]
		self.recording = False
		
		rospy.Subscriber("/eddy/joy", Joy, self.joy_callback)
		rospy.Subscriber("/rosbag/recording", Bool, self.record_callback)
		
		self.twist_pub = rospy.Publisher("/eddy/cmd_vel", Twist, queue_size=1)
		
		pass
	
	def record_callback(self, msg):
		self.recording = msg.data
		pass
	
	def joy_callback(self, msg):
		
		record_signal = msg.buttons[Controller.RECORD_BUTTON]
		
		if record_signal > 0 and not self.recording:
			rospy.wait_for_service('/rosbag/start', rospy.Duration(0.1))
			try:
				record_start = rospy.ServiceProxy('/rosbag/start', Empty)
				record_start()
				pass
			except rospy.ServiceException as e:
				rospy.logerr("Service call failed: %s" % e)
				pass
			pass
		elif record_signal < 0 and self.recording:
			rospy.wait_for_service('/rosbag/stop', rospy.Duration(0.1))
			try:
				record_stop = rospy.ServiceProxy('/rosbag/stop', Empty)
				record_stop()
				pass
			except rospy.ServiceException as e:
				rospy.logerr("Service call failed: %s" % e)
				pass
			pass
			
		twist = Twist()
		
		if msg.buttons[Controller.ARMED_BUTTON] <= 0:
			self.twist_pub.publish(twist)
			return
		
		trust_signal = msg.axes[Controller.THRUST_AXIS]
		steer_signal = msg.axes[Controller.STEER_AXIS]
		
		if trust_signal > 0:
			twist.linear.x = self.max_forward_vel * trust_signal / self.max_forward_vel
			pass
		elif trust_signal < 0:
			twist.linear.x = self.max_reverse_vel * trust_signal / self.max_reverse_vel
			pass
		
		twist.angular.z = self.max_rotational_vel * steer_signal / self.max_rotational_vel
		
		self.twist_pub.publish(twist)
		pass

def main(args):
	# Initialize the node
	rospy.init_node('teleop', anonymous=False)

	arg_defaults = {
		"max_forward_vel" : 2.0,
		"max_reverse_vel" : 1.5,
		"max_rotational_vel" : 1.0,
	}
	args = updateArgs(arg_defaults)
	
	teleop = Teleop(**args)
	pass

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
