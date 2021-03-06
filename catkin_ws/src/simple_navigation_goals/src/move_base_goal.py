#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

class movebase_simple(object):
	def __init__(self):
		self.goal = PoseStamped()
		self.pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
		self.sub = rospy.Subscriber("robot_point",Point,self.pointCallback,queue_size=1000)
		self.x = 0;
		self.y = 0;

	def movebase_simple(self):
		self.goal.header.frame_id = "map"
		self.goal.header.stamp = rospy.Time.now()
		self.goal.pose.position.x = self.x
		self.goal.pose.position.y = self.y
		self.goal.pose.orientation.w = 1.0
		rospy.sleep(1)
		self.pub.publish(self.goal)

	def pointCallback(self, msg):
		self.x = msg.x*0.1
		self.y = msg.y*0.1
		self.movebase_simple()
		rospy.loginfo("%lf,%lf",msg.x,msg.y)
		rospy.loginfo("%lf,%lf",self.x,self.y)

	def on_shutdown(self):
		rospy.logwarn("Stop goal")

if __name__ == '__main__':
	rospy.init_node('movebase_simple')
	node = movebase_simple()
	#node.movebase_simple()
	rospy.on_shutdown(node.on_shutdown)
	rospy.spin()
