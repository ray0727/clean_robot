#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

class movebase_simple(object):
	def __init__(self):
		self.goal = PoseStamped()
		self.pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
	def movebase_simple(self):
		self.goal.header.frame_id = "map"
		self.goal.header.stamp = rospy.Time.now()
		self.goal.pose.position.x = 0.5
		self.goal.pose.position.y = 0.5
		self.goal.pose.orientation.w = 0.5
		rospy.sleep(1)
		self.pub.publish(self.goal)

	def on_shutdown(self):
		rospy.logwarn("Stop goal")

if __name__ == '__main__':
	rospy.init_node('movebase_simple')
	node = movebase_simple()
	node.movebase_simple()
	rospy.on_shutdown(node.on_shutdown)

