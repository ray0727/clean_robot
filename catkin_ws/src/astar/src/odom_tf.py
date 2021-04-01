#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
import rospkg

from nav_msgs.msg import Odometry

def cb_odom(msg):
    
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z),
        (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
        msg.header.stamp,"base_link","odom")
    
if __name__ == '__main__':
    rospy.init_node('odom_tf')
    odom_sub = rospy.Subscriber(
            "/jackal_velocity_controller/odom", Odometry, cb_odom, queue_size=1)
    # husky_velocity_controller/odom
    rospy.spin()
