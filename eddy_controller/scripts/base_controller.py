#!/usr/bin/env python3

import sys
from math import cos, sin

from simple_pid import PID

### ROS imports

import rospy
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

def deadzone(value, pos_low_bound, neg_high_bound):
	if value > 0 and value > pos_low_bound:
		return 0
	elif value < 0 and value < neg_high_bound:
		return 0
	return value

def clamp(value, low, high):
	return max(low, min(value, high))

def Controller:

	CONTROL_NONE = 0
	CONTROL_TWIST = 0
	CONTROL_TWIST_LINEAR = 0

	def __init__(self, **params):
		self.control_mode = CONTROL_NONE
		
		self.max_forward_vel = params["max_forward_vel"]
		self.max_forward_force = params["max_forward_force"]
		self.max_reverse_vel = params["max_reverse_vel"]
		self.max_reverse_force = params["max_reverse_force"]
		self.max_rotational_vel = params["max_rotational_vel"]
		
		self.vel_measure = 0
		self.vel_kf = params["vel_kf"]
		self.vel_pid = PID(params["vel_kp"], params["vel_kd"], params["vel_ki"])
		self.vel_pid.sample_time = 1 / 20.0
		self.vel_cmd = 0
		
		self.rot_measure = 0
		self.rot_kf = param["rot_kf"]
		self.rot_pid = PID(params["rot_kp"], params["rot_kd"], params["rot_ki"])
		self.rot_pid.sample_time = 1 / 20.0
		self.rot_cmd = 0
		
		self.vel_cov_limit = params["vel_cov_limit"]
		self.vel_data_timeout = params["imu_data_timeout"]
		self.vel_data_time = 0
		
		self.imu_cov_limit = params["imu_cov_limit"]
		self.imu_data_timeout = params["imu_data_timeout"]
		self.imu_data_time = 0
		
		self.cmd_time = 0
		self.cmd_timeout = params["cmd_timeout"]
		
		self.wrench = Wrench()
		
		self.wrench.force.x = 0
		self.wrench.force.y = 0
		self.wrench.force.z = 0
		
		self.wrench.torque.x = 0
		self.wrench.torque.y = 0
		self.wrench.torque.z = 0
		
		rospy.Subscriber("/eddy/cmd_vel", Twist, self.cmd_vel_callback)
		rospy.Subscriber("/eddy/odom/filtered", Odometry, self.odom_callback)
		
		self.wrench_pub = rospy.Publisher("/eddy/cmd_thrust", Wrench, queue_size=1)
		pass
	
	def velocity_mapping(self):
		if self.vel_cmd > 0:
			self.wrench.force.x = self.max_forward_force * self.vel_cmd / self.max_forward_vel
			pass
		elif self.vel_cmd < 0:
			self.wrench.force.x = self.max_reverse_force * self.vel_cmd / self.max_reverse_vel
			pass
		self.wrench.force.x = deadzone(self.wrench.force.x, self.max_forward_force * 0.05, self.max_reverse_force * 0.05)
		pass
	
	def update_velocity_control(self):
		vel_error = self.vel_cmd - self.vel_measure
		vel_output = self.vel_pid(vel_error)
		vel_output = vel_output + self.vel_kf*self.vel_cmd
		
		self.wrench.force.x = deadzone(vel_output, self.max_forward_force * 0.05, self.max_reverse_force * 0.05)
		pass
	
	def update_rotational_control(self):
		rot_error = self.rot_cmd - self.rot_measure
		rot_output = self.rot_pid(rot_error)
		rot_output = rot_output + self.rot_kf*self.rot_cmd
		
		self.wrench.torque.z = deadzone(rot_output, 2, -2)
		pass
	
	def cmd_vel_callback(self, msg):
		self.rot_cmd = clamp(msg.angular.z, -self.max_rotational_vel, self.max_rotational_vel)
		self.update_rotational_control()
		
		self.vel_cmd = clamp(msg.linear.z, self.max_reverse_vel, self.max_forward_vel)
		if self.control_mode == CONTROL_TWIST_LINEAR:
			self.velocity_mapping()
			pass
		elif self.control_mode == CONTROL_TWIST:
			self.update_velocity_control()
			pass
		
		self.cmd_time = rospy.Time.now()
		pass
	
	def odom_callback(self, msg):
		
		#check if navsat/vel is being integrated into odometry
		if msg.twist.covariance[0] < self.vel_cov_limit and msg.twist.covariance[7] < self.vel_cov_limit:
			self.vel_data_time = rospy.Time.now().toSec()

		# check if imu/data is being integrated into odometry
		if msg.pose.covariance[35] < self.imu_cov_limit and msg.twist.covariance[35] < self.imu_cov_limit:
			self.imu_data_time = rospy.Time.now().toSec()
		
		quat = msg.pose.pose.orientation
		(_, _, self.yaw_measure) = euler_from_quaternion(quaternion=(quat.x, quat.y, quat.z, quat.w))
		self.rot_measure = msg.twist.twist.angular.z;
		
		vec = msg.twist.twist.linear
		self.vel_measure = vec.x * cos(self.yaw_measure) + vec.y * sin(self.yaw_measure)
		
		if self.control_mode == CONTROL_TWIST:
			self.update_velocity_control()
			self.update_rotational_control()
			pass
		elif self.control_mode == CONTROL_TWIST_LINEAR:
			self.update_rotational_control()
			pass
		pass
	
	def controller_update(self, event):
		
		self.vel_timeout = event.current_real - self.vel_data_time > self.vel_data_timeout
		self.imu_timeout = event.current_real - self.imu_data_time > self.imu_data_timeout
		
		if event.current_real - self.cmd_time > self.cmd_timeout:
			self.control_mode = CONTROL_NONE
			self.wrench.force.x = 0
			self.wrench.torque.z = 0
		elif self.vel_timeout or self.imu_timeout:
			self.control_mode = CONTROL_TWIST
			pass
		elif not self.imu_timeout:
			self.control_mode = CONTROL_TWIST_LINEAR
			pass
		
		self.wrench_pub.publish(self.wrench)
		pass

def main(args):
	# Initialize the node
	rospy.init_node('base_controller', anonymous=False)

	arg_defaults = {
		"max_forward_vel" : 2.0,
		"max_forward_force" : 4.52,
		"max_reverse_vel" : 1.5,
		"max_reverse_force" : -3.52,
		"max_rotational_vel" : 1.0,
		"vel_kf" : 10.0,
		"vel_kp" : 90.0,
		"vel_kd" : 1.0,
		"vel_ki" : 0.0,
		"rot_kf" : 10.0,
		"rot_kp" : 2.0,
		"rot_kd" : 1.0,
		"rot_ki" : 0.0,
		"vel_cov_limit" : 0.28,
		"vel_data_timeout" : 0.2,
		"imu_cov_limit" : 1.0,
		"imu_data_timeout" : 0.2,
		"cmd_timeout" : 0.5
	}
	args = updateArgs(arg_defaults)
	
	controller = Controller(**args)
	
	rospy.Timer(rospy.Duration(1/50.0), controller.controller_update)
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
