#!/usr/bin/env python

import rospy
from pure_pursuit import PurePursuit
from astar.srv import GoToPos, GoToPosResponse, GoToPosRequest
from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from visualization_msgs.msg import Marker
from subt_msgs.msg import ArtifactPoseArray, ArtifactPose
from pozyx_ros.msg import DeviceRangeArray, DeviceRange
import tf
import numpy as np
import math
from std_msgs.msg import Float32,Int32,String, Bool
from sensor_msgs.msg import Joy, JointState



class Navigation(object):
    def __init__(self):
        rospy.loginfo("Initializing %s" % rospy.get_name())

        ## parameters

        self.switch_thred = 1.8

        self.map_frame = rospy.get_param("~map_frame", 'odom')
        self.bot_frame = 'base_link'
        self.switch_thred = 1 # change to next lane if reach next

        self.pursuit = PurePursuit()
        self.pursuit.set_look_ahead_distance(0.7)

        self.set_lane_lock = False

        self.target_global = None
        self.listener = tf.TransformListener()

        self.pub_goal_marker = rospy.Publisher("goal_marker", Marker, queue_size=1)
        self.pub_target_marker = rospy.Publisher("target_marker", Marker, queue_size=1)
        self.pub_pid_goal = rospy.Publisher("pid_control/goal", PoseStamped, queue_size=1)
        self.pub_robot_speech = rospy.Publisher("speech_case", String, queue_size=1)
        self.pub_robot_go = rospy.Publisher("robot_go", Bool, queue_size=1)
        rospy.sleep(1) # wait for the publisher to be set well


        # ## node path
        # while not rospy.has_param("/guid_path") and not rospy.is_shutdown():
        #     rospy.loginfo("Wait for /guid_path")
        #     rospy.sleep(0.5)
        # self.guid_path = rospy.get_param("/guid_path")
        # self.tag_ang_init = rospy.get_param("/guid_path_ang_init")
        # self.last_node = -1
        # self.next_node_id = None
        # self.all_anchor_ids = rospy.get_param("/all_anchor_ids")
        # self.joy_lock = False

        # self.get_goal = True
        # self.joint_ang = None

        self.joint_mode = None

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        sub_joy = rospy.Subscriber('joy_teleop/joy', Joy, self.cb_joy, queue_size=1)
        sub_joy = rospy.Subscriber('/joy', Joy, self.cb_joy, queue_size=1)
        sub_joint = rospy.Subscriber('/dynamixel_workbench/joint_states', JointState, self.cb_joint, queue_size=1)
        sub_fr = rospy.Subscriber('front_right/ranges', DeviceRangeArray, self.cb_range, queue_size=1)

        self.req_path_srv = rospy.ServiceProxy("plan_service", GetPlan)

        ## node path
        while not rospy.has_param("/guid_path") and not rospy.is_shutdown():
            rospy.loginfo("Wait for /guid_path")
            rospy.sleep(0.5)
        self.guid_path = rospy.get_param("/guid_path")
        self.tag_ang_init = rospy.get_param("/guid_path_ang_init")
        self.last_node = -1
        self.next_node_id = None
        self.all_anchor_ids = rospy.get_param("/all_anchor_ids")
        self.joy_lock = False
        self.last_pid_goal = None

        ## node location
        while not rospy.has_param('/guid_anchor_pos') and not rospy.is_shutdown():
            rospy.loginfo("Wait for guid_anchor_pos")
            rospy.sleep(0.5)
        ## transform goal to map frame
        guid_anchor_pos_local = rospy.get_param("/guid_anchor_pos")
        self.guid_anchor_pos = {}
        for anc in guid_anchor_pos_local:
            p = Pose()
            p.position.x = guid_anchor_pos_local[anc][0]
            p.position.y = guid_anchor_pos_local[anc][1]
            self.guid_anchor_pos[anc] = None
            while self.guid_anchor_pos[anc] is None:
                self.guid_anchor_pos[anc] = self.transform_pose(p, self.map_frame, self.bot_frame)
                rospy.loginfo("Wait tf for anchor position transformation")
                rospy.sleep(0.3)
            print(anc)
        self.goal_list = self.guid_anchor_pos

        self.last_plan = None
        self.search_angle = 10*math.pi/180
        self.search_max = 5

        print(self.goal_list)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tracking)

        self.set_lane(True)
        print("init done")


    def to_marker(self, goal, color=[0, 1, 0]):
        marker = Marker()
        marker.header.frame_id = goal.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.pose.position.x = goal.pose.position.x
        marker.pose.position.y = goal.pose.position.y
        marker.id = 0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        return marker

    def transform_pose(self, pose, target_frame, source_frame):
        try:
            (trans_c, rot_c) = self.listener.lookupTransform(
                target_frame, source_frame, rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("faile to catch tf %s 2 %s" %
                         (target_frame, source_frame))
            return

        trans_mat = tf.transformations.translation_matrix(trans_c)
        rot_mat = tf.transformations.quaternion_matrix(rot_c)
        tran_mat = np.dot(trans_mat, rot_mat)

        target_mat = np.array([[1, 0, 0, pose.position.x],
                               [0, 1, 0, pose.position.y],
                               [0, 0, 1, pose.position.z],
                               [0, 0, 0, 1]])

        target = np.dot(tran_mat, target_mat)
        quat = tf.transformations.quaternion_from_matrix(target)
        trans = tf.transformations.translation_from_matrix(target)

        t_pose = PoseStamped()
        t_pose.header.frame_id = target_frame
        t_pose.pose.position.x = trans[0]
        t_pose.pose.position.y = trans[1]
        t_pose.pose.position.z = trans[2]
        t_pose.pose.orientation.x = quat[0]
        t_pose.pose.orientation.y = quat[1]
        t_pose.pose.orientation.z = quat[2]
        t_pose.pose.orientation.w = quat[3]

        return t_pose

    def tracking(self, event):

        if self.target_global is None:
            rospy.logerr("%s : no goal" % rospy.get_name())
            return

        end_p = self.transform_pose(
            self.target_global.pose, self.bot_frame, self.map_frame)

        if end_p is None:
            return

        self.pub_target_marker.publish(self.to_marker(end_p, [0, 0, 1]))        

        d = math.sqrt(end_p.pose.position.x**2 + end_p.pose.position.y**2)

        #if d <= 0.8:
        #    rospy.logwarn("goal reached0")
        #    self.target_global = None
        #    self.next_goal()
        #    return

        start_p = PoseStamped()
        start_p.header = end_p.header
        start_p.pose.position.x = 0
        start_p.pose.position.y = 0
        start_p.pose.position.z = 0

        req_path = GetPlanRequest()
        req_path.start = start_p
        req_path.goal = end_p

        
        ## in case map is not accurate and no path to the real goal point
        oe = end_p
        for i in range(self.search_max):
            try:
                res_path = self.req_path_srv(req_path)
                self.last_plan = res_path
                #rospy.loginfo("%s : plan success" % rospy.get_name())
                break
            except:
                rospy.logerr("%s : path request fail" % rospy.get_name())
                if self.last_plan is None:
                    return
                res_path = self.last_plan
                r = np.linalg.norm([oe.pose.position.x, oe.pose.position.y])
                theta = math.atan2(oe.pose.position.y, oe.pose.position.x)
                theta += (-1)**(i) * (i+1) * self.search_angle
                end_p.pose.position.x = r * math.cos(theta)
                end_p.pose.position.y = r * math.sin(theta)

        self.pursuit.set_path(res_path.plan)
        goal = self.pursuit.get_goal(start_p)

        if goal is None:
            rospy.logwarn("goal reached1")
            if self.last_pid_goal != None :
                self.pub_pid_goal.publish(self.last_pid_goal)
            else :
                rospy.logerr("GGGGGGGGGGGGGGGGGGGGGGGGGGGGG")
            return

        # rospy.loginfo(goal.pose.position)

        # goal = self.transform_pose(goal.pose, "map", "base_link")
        goal.header.frame_id = 'base_link'
        
        self.pub_goal_marker.publish(self.to_marker(goal))

        self.pub_pid_goal.publish(goal)
        self.last_pid_goal = goal

    def next_goal(self):
        if len(self.goal_list) == 0:
            print("ENDING")
            return

        rospy.loginfo('~~~Next goal~~~')
        self.target_global = None
        rospy.sleep(3)
        # self.set_lane(True)
        self.target_global = self.goal_list.pop(0)



    def set_lane(self, next):

        if self.set_lane_lock:
            print("set lane lock")
            return

        self.set_lane_lock = True

        #self.pub_robot_go.publish(False)

        # to next node
        if next:
            self.last_node += 1
        else:
            self.last_node -= 1

        # select sound file
        s_anchor = str(self.guid_path[self.last_node])
        sound_file = str(self.guid_path[self.last_node]) + '_2'
        print("robot sound:", sound_file)

        ###### head and toe #####################
        if self.last_node >= len(self.guid_path)-1:
            rospy.loginfo("It's the last lane.")
           
            self.pub_robot_go.publish(False)

            # play sound on robot
            self.pub_robot_sound(sound_file)
            # this make anchorball speack
            # see pozyx_ros/src/multi_ranging_guid.py
            rospy.set_param("/guid_lane_next", 0)
            #rospy.set_param("/velocity_mode", 0)

            self.last_node -= 1

            self.set_lane_lock = False
            return

        elif self.last_node < 0:
            rospy.loginfo("Back to first lane.")
            self.last_node = 0
        ##########################################

        # play sound on robot
        self.pub_robot_sound(sound_file)

        # set pozyx ranging tag id
        pozyx_id = {}
        pozyx_id[self.guid_path[self.last_node]] = self.all_anchor_ids[self.guid_path[self.last_node]]
        pozyx_id[self.guid_path[self.last_node+1]] = self.all_anchor_ids[self.guid_path[self.last_node+1]]
        try:
            rospy.delete_param("/anchorballs")
            rospy.set_param("/anchorballs", pozyx_id)
        except KeyError:
            rospy.logerr("No such param /anchorballs")

        # set last, next node (anchor)
        # this make anchorball speack
        # see pozyx_ros/src/multi_ranging_guid.py
        # next node id
        self.next_node_id = self.all_anchor_ids[self.guid_path[self.last_node+1]]
        rospy.sleep(1)
        rospy.set_param("/guid_lane_last", self.all_anchor_ids[self.guid_path[self.last_node]])
        rospy.set_param("/guid_lane_next", self.all_anchor_ids[self.guid_path[self.last_node+1]])
        # start turning (right, left or back)
        
        self.special_case()

        # set new goal
        try:
            self.target_global = self.guid_anchor_pos[self.guid_path[self.last_node+1]]
        except KeyError:
            rospy.logerr("There's no position data for " + self.guid_path[self.last_node+1])
        self.pub_robot_go.publish(True)

        # to wait for everything to reset
        # self.target_global = None

        self.set_lane_lock = False
    
    def special_case(self):
        # we hard code turning situation here
        # not the best solution, but an achievable one at the moment

        if self.last_node == 0:
            return

        this_ang = self.tag_ang_init[self.guid_path[self.last_node]]

        if this_ang != 180:
            return
        else:
            self.pub_robot_go.publish(False)
            # 180 degree turn, stage 1, tie up harness

            rospy.sleep(2)
            lt = rospy.Time.now().to_sec()
            self.pub_robot_sound('tie_up')
            while self.joint_ang > math.radians(10):
                tt = rospy.Time.now().to_sec()
                if (tt-lt) >= 4:
                    # play sound on robot
                    self.pub_robot_sound('tie_up')
                    lt = tt
                rospy.sleep(0.1)
            
            # 180 degree turn, stage 2, turn 180
            turn_cmd = Twist()
            turn_cmd.angular.z = 0.5
            for i in range(700):
                self.pub_cmd.publish(turn_cmd)
                rospy.sleep(0.01)

            # 180 degree turn, stage 3, release harness
            lt = rospy.Time.now().to_sec()
            self.pub_robot_sound('release')
            while self.joint_ang < math.radians(40):
                tt = rospy.Time.now().to_sec()
                if (tt-lt) >= 4:
                    # play sound on robot
                    self.pub_robot_sound('release')
                    lt = tt
                rospy.sleep(0.1)
            self.pub_robot_go.publish(True)

    def pub_robot_sound(self, data):

        # play sound on robot
        str_msg = String()
        str_msg.data = data
        self.pub_robot_speech.publish(str_msg)


    def cb_joy(self, msg):
        
        #print("cb_joy")
        #switch = 0
        #if msg.axes[-3] < -0.5:
        #    switch = 1
        #elif msg.axes[2] < -0.5:
        #    switch = -1

        #print("b:",msg.axes[-1])
        #print("f:",msg.axes[-2])
        switch = 0
        if msg.buttons[-1] == 1:
            switch = 1
        elif msg.buttons[-2] == 1:
            switch = -1

        if switch == 0:
            self.joy_lock = False

        if self.joy_lock:
            return
        
        if switch == 1:
            rospy.loginfo("Joy to the next lane.")
            self.set_lane(True)
            self.joy_lock = True
        elif switch == -1:
            rospy.loginfo("Joy to the last lane.")
            self.set_lane(False)
            self.joy_lock = True
    
    def cb_joint(self, msg):

        if self.joint_mode is None:
            if msg.position[0] < 0:
                self.joint_mode = 1
            else:
                self.joint_mode = 2

        if self.joint_mode == 1:
            self.joint_ang = -1*msg.position[0] - math.radians(180)
        else:
            self.joint_ang = -1*msg.position[0] + math.radians(180)

    def cb_range(self, msg):

        if len(msg.rangeArray) == 0:
            return

        d = msg.rangeArray[0].distance / 1000.
        tid = msg.rangeArray[0].tag_id

        if tid != self.next_node_id:
            return

        # if d < 10cm, it's probably
        if d < 0.1:
            return


        if d <= self.switch_thred:
            rospy.logwarn("goal reached, to next goal")
            self.target_global = None
            self.set_lane(True)
            return


if __name__ == "__main__":
    rospy.init_node("navigation")
    nav = Navigation()
    rospy.spin()
