#! /usr/bin/env python3
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String

GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN)
class gpio_bump(object):
	def __init__(self):
		self.pub = rospy.Publisher("bump_detect", String, queue_size=1)
		self.data = String()
		self.previous = "LOW"
		self.current = "LOW"
	def detecting(self):
		while not rospy.is_shutdown():
			if GPIO.input(21):
				self.data = "HIGH"
				self.previous = self.current
				self.current = "HIGH"
			else:
				self.data = "LOW"
				self.previous = self.current
				self.current = "LOW"
			if((self.previous=="LOW" and self.current=="HIGH") or (self.previous=="HIGH" and self.current=="LOW")):
				self.pub.publish(self.data)

	def on_shutdown(self):
		rospy.loginfo("shutdown")

if __name__ == "__main__":
	rospy.init_node('gpio_bump')
	bump_detect = gpio_bump() 
	bump_detect.detecting()
	rospy.on_shutdown(bump_detect.on_shutdown)
