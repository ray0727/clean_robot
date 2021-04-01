#!/usr/bin/env python2

import rospy
from pure_pursuit import PurePursuit
from astar.srv import GoToPos, GoToPosResponse, GoToPosRequest
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy, JointState
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from anchor_measure.msg import PoseDirectional
from visualization_msgs.msg import Marker
from subt_msgs.msg import ArtifactPoseArray, ArtifactPose
from pozyx_ros.msg import DeviceRangeArray, DeviceRange
import tf
import tf2_ros
import numpy as np
import math

# for using tf to draw range
r_tag_points = np.array([[-0.7, 0.0],[0.12, 0.13]])

class Navigation(object):
    def __init__(self):
        rospy.loginfo("Initializing %s" % rospy.get_name())

        ## parameters
        self.map_frame = rospy.get_param("~map_frame", 'odom')
        self.bot_frame = 'base_link'
        self.switch_thred = 1.5 # change to next lane if reach next

        self.pursuit = PurePursuit()
        self.lka = rospy.get_param("~lookahead", 0.5)
        self.pursuit.set_look_ahead_distance(self.lka)

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

        self.get_goal = True
        self.joint_ang = None

        ## set first tracking lane
        self.pub_robot_speech = rospy.Publisher("speech_case", String, queue_size=1)
        self.pub_robot_go = rospy.Publisher("robot_go", Bool, queue_size=1)
        rospy.sleep(1) # wait for the publisher to be set well
        self.set_lane(True)

        # variable
        self.target_global = None
        self.listener = tf.TransformListener()

        # markers
        self.pub_goal_marker = rospy.Publisher(
            "goal_marker", Marker, queue_size=1)

        self.pub_target_marker = rospy.Publisher(
            "target_marker", Marker, queue_size=1)

        # publisher, subscriber
        self.pub_pid_goal = rospy.Publisher(
            "pid_control/goal", PoseStamped, queue_size=1)
            
        
        self.req_path_srv = rospy.ServiceProxy("plan_service", GetPlan)

        sub_box = rospy.Subscriber(
            "anchor_position", PoseDirectional, self.cb_goal, queue_size=1)
        
        sub_joy = rospy.Subscriber('joy_teleop/joy', Joy, self.cb_joy, queue_size=1)
        sub_fr = rospy.Subscriber('front_right/ranges', DeviceRangeArray, self.cb_range, queue_size=1)
        sub_joint = rospy.Subscriber('/dynamixel_workbench/joint_states', JointState, self.cb_joint, queue_size=1)

        #Don't update goal too often
        self.last_update_goal = None
        self.goal_update_thred = 0.001
        self.hist_goal = np.array([])

        self.normal = True
        self.get_goal = True
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tracking)

        # print("init done")
        
        # prevent sudden stop
        self.last_plan = None
        # keep searching until find path or reach search end
        self.search_angle = 10*math.pi/180
        self.search_max = 5

        ### stupid range drawing
        # for using tf to draw range
        br1 = tf2_ros.StaticTransformBroadcaster()
        br2 = tf2_ros.StaticTransformBroadcaster()
        
        # stf.header.frame_id = "uwb_back_left"
        # stf.child_frame_id = "base_link"
        # stf.transform.translation.x = -1*r_tag_points[0, 0]
        # stf.transform.translation.y = -1*r_tag_points[0, 1]
        # br1.sendTransform(stf)

        # stf2 = stf
        # stf2.header.stamp = rospy.Time.now()
        # stf2.header.frame_id = "uwb_front_right"
        # stf2.transform.translation.x = -1*r_tag_points[1, 0]
        # stf2.transform.translation.y = -1*r_tag_points[1, 1]
        # br2.sendTransform(stf2)

        stf = TransformStamped()
        stf.header.stamp = rospy.Time.now()
        stf.transform.rotation.w = 1
        stf.header.frame_id = "base_link"
        stf.child_frame_id = "uwb_back_left"
        stf.transform.translation.x = r_tag_points[0, 0]
        stf.transform.translation.y = r_tag_points[0, 1]

        stf2 = TransformStamped()
        stf2.header.stamp = rospy.Time.now()
        stf2.transform.rotation.w = 1
        stf2.header.frame_id = "base_link"
        stf2.child_frame_id = "uwb_front_right"
        stf2.transform.translation.x = r_tag_points[1, 0]
        stf2.transform.translation.y = r_tag_points[1, 1]
        # br2.sendTransform([stf, stf2])

        # print(rospy.Time.now())

    def tracking(self, event):
        if not self.normal:
            return

        st = rospy.Time.now()
        #rospy.loginfo("tracking loop")

        if self.target_global is None:
            rospy.logerr("%s : no goal" % rospy.get_name())
            return
        else:
            #rospy.loginfo("%s :have seen goal" % rospy.get_name())
            pass

        #print("fuckkckckc why???")
        #print(self.target_global)

        end_p = self.transform_pose(self.target_global.pose, self.bot_frame, self.map_frame)
        #end_p = self.target_global
        #end_p.header.frame_id = self.map_frame
        if end_p is None:
            return
        end_p.header.frame_id = self.bot_frame
        

        start_p = PoseStamped()
        start_p.pose.position.x = 0
        start_p.pose.position.y = 0
        start_p.pose.position.z = 0
        start_p.header.frame_id = self.bot_frame
        #start_p = self.transform_pose(start_p.pose, self.map_frame, self.bot_frame)
        if start_p is None:
            return

        req_path = GetPlanRequest()
        req_path.start = start_p
        req_path.goal = end_p

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
        
        self.pub_target_marker.publish(self.to_marker(end_p, [0, 0, 1]))
                

        self.pursuit.set_path(res_path.plan)

        goal = self.pursuit.get_goal(start_p)

        if goal is None:
            rospy.logwarn("goal reached, to next goal")
            self.target_global = None
            self.set_lane(True)
            return
        # print("trace", goal)
        goal = self.transform_pose(goal.pose, self.map_frame, self.bot_frame)
        goal.header.frame_id = self.map_frame

        self.pub_goal_marker.publish(self.to_marker(goal))

        self.pub_pid_goal.publish(goal)

        et = rospy.Time.now()
        #print("Duration:", et.to_sec()-st.to_sec())
        #print("============================================")

    def cb_goal(self, msg):

        if not self.get_goal:
            return

        self.bot_frame = msg.header.frame_id
        now_t = rospy.Time.now()
        if self.last_update_goal is None:
            self.target_global = msg.pose # posestamped
            self.target_global = self.transform_pose(msg.pose.pose, self.map_frame, msg.header.frame_id, msg.header.stamp)
            if self.target_global is None:
                return
            self.target_global.header.frame_id = self.map_frame
            return

        dt = now_t.to_sec() - self.last_update_goal.to_sec()
        
        if dt >= self.goal_update_thred:
            tg = self.transform_pose(msg.pose.pose, self.map_frame, msg.header.frame_id, msg.header.stamp)
            if tg is None:
                return
            tg.header.frame_id = self.map_frame
            self.hist_goal = np.append(self.hist_goal, tg)

            self.target_global = tg
            self.hist_goal = np.array([])

        else:
            tg = self.transform_pose(msg.pose.pose, self.map_frame, msg.header.frame_id, msg.header.stamp)
            if tg is None:
                return
            tg.header.frame_id = self.map_frame
            self.hist_goal = np.append(self.hist_goal, tg)
        self.last_update_goal = now_t
    
    def cb_range(self, msg):

        if len(msg.rangeArray) == 0:
            return
        d = (msg.rangeArray[0].distance)/1000.
        tid = msg.rangeArray[0].tag_id

        #print("d:", d)
        #print("tid:",tid)
        #print("next:",self.next_node_id)

        if tid != self.next_node_id:
            return
        
        # if d < 10cm, it's probably
        if d < 0.1: 
            return

        #print("d:",d)
        #print("self.switch:",self.switch_thred)
        #print("")
        if d <= self.switch_thred:
            rospy.logwarn("goal reached, to next goal")
            self.target_global = None
            self.set_lane(True)
            return
        #print(self.target_global)

    
    def cb_joy(self, msg):
        
        #print("cb_joy")
        #switch = 0
        #if msg.axes[-3] < -0.5:
        #    switch = 1
        #elif msg.axes[2] < -0.5:
        #    switch = -1

        #print("b:",msg.axes[-1])
        #print("f:",msg.axes[-2])
        switch = None
        if msg.buttons[-1] == 1 and msg.buttons[-2] == 0:
            switch = -1
        elif msg.buttons[-1] == 0 and msg.buttons[-2] == 1:
            switch = 1
        elif msg.buttons[-1] == 1 and msg.buttons[-2] == 1:
            return
        else:
            switch = 0

        # switch = msg.axes[-1]
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

        self.joint_ang = -1*msg.position[0]
    
    def set_lane(self, next):

        self.pub_robot_go.publish(False)

        # to next node
        if next:
            self.last_node += 1
        else:
            self.last_node -= 1

        # select sound file
        s_anchor = str(self.guid_path[self.last_node])
        if len(s_anchor) <= 9:
            sound_file1 = str(self.guid_path[self.last_node]) + '_1'
            sound_file2 = str(self.guid_path[self.last_node]) + '_2'
        else:
            sound_file1 = str(self.guid_path[self.last_node])[:-2] + '_3'
            sound_file2 = str(self.guid_path[self.last_node])[:-2] + '_4'
        
        ###### head and toe #####################
        if self.last_node >= len(self.guid_path)-1:
            rospy.loginfo("It's the last lane.")
            
            # play sound on robot
            self.pub_robot_sound(sound_file1)
            rospy.sleep(2)
            # this make anchorball speack
            # see pozyx_ros/src/multi_ranging_guid.py
            rospy.set_param("/guid_lane_next", 0)
            rospy.set_param("/velocity_mode", 0)

            self.last_node -= 1
            return
        elif self.last_node < 0:
            rospy.loginfo("Back to first lane.")
            self.last_node = 0
        ##########################################

        # play sound on robot
        self.pub_robot_sound(sound_file1)
        rospy.sleep(2)

        # set pozyx ranging tag id
        try:
            rospy.delete_param("/anchorballs")
        except KeyError:
            rospy.logerr("No such param /anchorballs")
        pozyx_id = {}
        pozyx_id[self.guid_path[self.last_node]] = self.all_anchor_ids[self.guid_path[self.last_node]]
        pozyx_id[self.guid_path[self.last_node+1]] = self.all_anchor_ids[self.guid_path[self.last_node+1]]
        rospy.set_param("/anchorballs", pozyx_id)

        # set last, next node (anchor)
        # this make anchorball speack
        # see pozyx_ros/src/multi_ranging_guid.py
        # next node id
        self.target_global = None
        self.next_node_id = self.all_anchor_ids[self.guid_path[self.last_node+1]]
        rospy.set_param("/guid_lane_last", self.all_anchor_ids[self.guid_path[self.last_node]])
        rospy.set_param("/guid_lane_next", self.all_anchor_ids[self.guid_path[self.last_node+1]])
        # start turning (right, left or back)
        self.normal = False
        rospy.sleep(2)
        self.special_case()

        self.pub_robot_go.publish(True)
        # play sound on robot
        self.pub_robot_sound(sound_file2)
        rospy.sleep(5)

        # to wait for everything to reset
        # self.target_global = None
    
    def special_case(self):
        # we hard code turning situation here
        # not the best solution, but an achievable one at the moment

        if self.last_node == 0:
            self.normal=True
            return

        this_ang = self.tag_ang_init[self.guid_path[self.last_node]]

        if this_ang != 180:
            self.get_goal = False
            self.target_global = None
            while self.target_global is None:
                self.target_global = PoseStamped()
                self.target_global.pose.position.x = 5*math.cos(math.radians(this_ang))
                self.target_global.pose.position.y = 5*math.cos(math.radians(this_ang))
                self.target_global = self.transform_pose(self.target_global.pose, self.map_frame, self.bot_frame)
            self.target_global.header.frame_id = self.map_frame
            self.normal = True
            self.pub_robot_go.publish(True)
            rospy.sleep(5)
            self.get_goal = True
        else:
            self.get_goal = False
            # 180 degree turn, stage 1, tie up harness
            while self.joint_ang > math.radians(5):
                # play sound on robot
                self.pub_robot_sound('tie_up')
                rospy.sleep(4)
            
            # 180 degree turn, stage 2, turn 180
            turn_goal = None
            print("turn_goal")
            while turn_goal is None:
                turn_goal = PoseStamped()
                turn_goal.pose.position.x = -5
                turn_goal_bot = turn_goal
                turn_goal_pid = turn_goal
                turn_goal_pid.pose.position.x = -0.05
                turn_goal_pid.pose.position.y = -0.005
                turn_goal = self.transform_pose(turn_goal.pose, self.map_frame, self.bot_frame)
            turn_goal_dummy = turn_goal
            self.pub_robot_go.publish(True)
            print("after")
            while math.fabs(math.atan2(turn_goal_bot.pose.position.y, turn_goal_bot.pose.position.x)) >= math.radians(90):
                print("pub turn goal")
                self.pub_pid_goal.publish(self.transform_pose(turn_goal_pid.pose, self.map_frame, self.bot_frame))
                rospy.sleep(0.1)
                turn_goal_bot = self.transform_pose(turn_goal.pose, self.bot_frame, self.map_frame)
                print(math.fabs(math.atan2(turn_goal_bot.pose.position.y, turn_goal_bot.pose.position.x)))
            self.pub_robot_go.publish(False)

            # 180 degree turn, stage 3, release harness
            while self.joint_ang < math.radians(45):
                # play sound on robot
                self.pub_robot_sound('release')
                rospy.sleep(4)
            
            self.normal = True
            self.get_goal = True

    def pub_robot_sound(self, data):

        # play sound on robot
        str_msg = String()
        str_msg.data = data
        self.pub_robot_speech.publish(str_msg)
    
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
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        return marker

    def transform_pose(self, pose, target_frame, source_frame, stamp=None):
        
        # print("Test rospy time 0", rospy.Time(0).to_sec())
        # print("Test msg time", stamp.to_sec())
        #print(target_frame)
        #print(source_frame)

        if target_frame is None or source_frame is None:
            return

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

        # print(trans_c)

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


if __name__ == "__main__":
    rospy.init_node("navigation_guid")
    nav = Navigation()
    rospy.spin()
