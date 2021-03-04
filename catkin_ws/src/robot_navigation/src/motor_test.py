#! /usr/bin/env python3
import time
import board
from adafruit_motorkit import MotorKit
import sys
import rospy
from nav_msgs.msg import Odometry

class motor_test(object):
	def __init__(self):
		self.right=0.3
		self.left=0.3
		self.sub = rospy.Subscriber("odom", Odometry, self.callback, queue_size=1)
		self.kit = MotorKit(i2c=board.I2C())
		self.oldright=0.1;
		self.oldleft=0.3;
		self.num=0

	def callback(self,data):
		
		if self.num==4:
			self.num=0
		
			
			
			#self.num=self.num+1;
			if data.pose.pose.orientation.z>0.01:
				self.right=self.right-0.05*data.pose.pose.orientation.z
				#self.left=self.left+0.01
			elif data.pose.pose.orientation.z<-0.01:
				self.right=self.right-0.05*data.pose.pose.orientation.z
				#self.left=self.left-0.01
		
			if self.right>=3.1:
				self.right=0.31
			elif self.right<=2.9:
				self.left=0.29

			if self.right!=self.oldright or self.left!=self.oldleft:
				rospy.loginfo("left:%lf",self.left)
				rospy.loginfo("right:%lf",self.right)
				self.motor_run()
		else:
			self.num=self.num+1
	
	def motor_run(self):
		if(self.right!=self.oldright):
			self.kit.motor1.throttle = self.left #stop motor
			self.kit.motor2.throttle = self.right
			self.oldright=self.right

	def on_shutdown(self):
		rospy.logwarn("Stop motors")
		self.kit.motor1.throttle=0
		self.kit.motor2.throttle=0


if __name__ == '__main__':
	rospy.init_node('motor_test_node',anonymous= False) #if anonynous is true,Ros requires that each have a unique name
	node = motor_test()
	rospy.on_shutdown(node.on_shutdown)
	rospy.spin()


'''def screen_control():
		while True:
			s=input('please input what level speed you want?:\n')
			if s=="1":#forward
				self.kit.motor1.throttle=0.24
				self.kit.motor2.throttle=0.2
			elif s=="2":#backward
				self.kit.motor1.throttle=-0.2
				self.kit.motor2.throttle=-0.215
			elif s=="3":#right	
				self.kit.motor1.throttle=0.2
				self.kit.motor2.throttle=-0.2
			elif s=="4":#left
				self.kit.motor1.throttle=-0.2
				self.kit.motor2.throttle=0.2
			elif s=="5":#fly
				self.kit.motor1.throttle=0.5
				self.kit.motor2.throttle=0.5
			elif s=="stop":
				self.kit.motor1.throttle = 0 #stop motor
				self.kit.motor2.throttle = 0
				break
			else:
				self.kit.motor1.throttle = 0 #stop motor
				self.kit.motor2.throttle = 0'''
		

