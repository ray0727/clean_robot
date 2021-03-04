#! /usr/bin/env python3
import time
import board
from adafruit_motorkit import MotorKit
import sys
kit =  MotorKit(i2c=board.I2C())
import rospy
from nav_msgs.msg import Odometry
import clrobot_self as self
a=self.Remote()
print(a.get_state())




right=0.3
left=0.3
def callback(data):
	if data.pose.pose.orientation.z>0.05:
		right=right-0.01;
	elif data.pose.pose.orientation.z<-0.05:
		right=right+0.01
	motor_run();
	
def screen_control():
	while True:
		s=input('please input what level speed you want?:\n')
		'''if (float(s)<=1.0) and (float(s)>=-1.0): #if s>0 forward,s<0 backward
			kit.motor1.throttle = float(s)
			time.sleep(2) #two second
			print('down')
			kit.motor1.throttle = 0
		elif s == 'exit':
			break
		else:
	 		print('error input')'''
		if s=="1":#forward
			kit.motor1.throttle=0.3
			kit.motor2.throttle=0.4
		elif s=="2":#backward
			kit.motor1.throttle=-0.2
			kit.motor2.throttle=-0.215
		elif s=="3":#right	
			kit.motor1.throttle=0.2
			kit.motor2.throttle=-0.2
		elif s=="4":#left
			kit.motor1.throttle=-0.2
			kit.motor2.throttle=0.2
		elif s=="5":#fly
			kit.motor1.throttle=0.5
			kit.motor2.throttle=0.5
		elif s=="stop":
			kit.motor1.throttle = 0 #stop motor
			kit.motor2.throttle = 0
			break
		else:
			kit.motor1.throttle = 0 #stop motor
			kit.motor2.throttle = 0
		
	kit.motor1.throttle = 0 #stop motor
	kit.motor2.throttle = 0

def motor_run():
	kit.motor1.throttle = left #stop motor
	kit.motor2.throttle = right

if __name__ == '__main__':
	screen_control()
	
	#rospy.spin();
