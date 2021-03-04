#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
import time
import board
from adafruit_motorkit import MotorKit
import sys

class ControlNode(object):
	def __init__(self):
		self.kit =  MotorKit(i2c=board.I2C())
		self.sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, self.cmdCallback, queue_size = 1)
		self.move_cmd = Twist()

	def cmdCallback(self, msg):
		self.move_cmd = msg
		speed = self.move_cmd.linear.x
		angular = self.move_cmd.angular.z
		straight_param = 0.3

		if(speed<=0.001 and speed>=-0.001 and angular==0):
			self.kit.motor1.throttle= 0
			self.kit.motor2.throttle= 0

		elif(speed<=-0.05):
			self.kit.motor1.throttle=0.3
			self.kit.motor2.throttle=-0.3
		else:
			#straight
			if(angular<=straight_param and angular>=-straight_param):
				self.kit.motor1.throttle=0.3
				self.kit.motor2.throttle=0.25
			#turn right
			elif(angular<-straight_param):
				self.kit.motor1.throttle=0.25
				self.kit.motor2.throttle=-0.25
			#turn left
			elif(angular>straight_param):
				self.kit.motor1.throttle=-0.25
				self.kit.motor2.throttle=0.25

	def on_shutdown(self):
		rospy.logwarn("Stopping motors")
		self.kit.motor1.throttle = 0
		self.kit.motor2.throttle = 0

if __name__ == "__main__":
	rospy.init_node("control_node", anonymous = False)
	node = ControlNode()
	rospy.on_shutdown(node.on_shutdown)
	rospy.spin()

