#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid


class PubMap(object):
    def __init__(self):
        rospy.loginfo("init %s" % rospy.get_name())

        self.fix_map = OccupancyGrid()
        self.fix_map.header.frame_id = "global"
        self.fix_map.info.map_load_time = rospy.Time.now()

        map_x = float(rospy.get_param("~map_x", 12))
        map_y = float(rospy.get_param("~map_y", 12))

        self.fix_map.info.resolution = 0.6
        self.fix_map.info.height = int(map_y / self.fix_map.info.resolution)
        self.fix_map.info.width = int(map_x / self.fix_map.info.resolution)
        self.fix_map.info.origin.position.x = -1.8
        self.fix_map.info.origin.position.y = -1.8
        self.fix_map.info.origin.position.z = 0
        self.fix_map.info.origin.orientation.w = 1

        obs_list = [[-0.9, -0.9, 1.8, 1.8]]

        tmp_map = np.zeros(
            [self.fix_map.info.width, self.fix_map.info.height], dtype=np.int16)
        
        for obs in obs_list:
            obs_x_s = (obs[0] - self.fix_map.info.origin.position.x)/self.fix_map.info.resolution
            obs_y_s = (obs[1] - self.fix_map.info.origin.position.y)/self.fix_map.info.resolution
            obs_x_e = (obs[0]+obs[2] - self.fix_map.info.origin.position.x)/self.fix_map.info.resolution
            obs_y_e = (obs[1]+obs[3] - self.fix_map.info.origin.position.y)/self.fix_map.info.resolution
        
            tmp_map[int(obs_x_s)-1:int(obs_x_e)+1, int(obs_y_s)-1:int(obs_y_e)+1] = 20
            tmp_map[int(obs_x_s):int(obs_x_e), int(obs_y_s):int(obs_y_e)] = 99
        
        tmp_map = np.reshape(tmp_map, -1)

        self.fix_map.data = tmp_map.tolist()

        self.pub_map = rospy.Publisher("map", OccupancyGrid, queue_size=1)
        timer = rospy.Timer(rospy.Duration(1), self.cb_pub)

    def cb_pub(self, event):
        self.fix_map.header.stamp = rospy.Time.now()
        self.pub_map.publish(self.fix_map)


if __name__ == "__main__":
    rospy.init_node("pub_map")
    pubmap = PubMap()
    rospy.spin()
