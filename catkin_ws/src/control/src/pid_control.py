#!/usr/bin/env python
import numpy as np
import rospy
import tf
import math
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_reconfigure.server import Server
from control.cfg import pos_PIDConfig, ang_PIDConfig, pos_loco_PIDConfig, ang_loco_PIDConfig, pos_jackal_PIDConfig, ang_jackal_PIDConfig
from std_srvs.srv import SetBool, SetBoolResponse
from nav_msgs.msg import Odometry
from PID import PID_control
from std_msgs.msg import String, Bool
from copy import deepcopy

class Robot_PID():
    def __init__(self):

        cmd_ratio = rospy.get_param("~cmd_ratio", 1)

        self.dis4constV = 1.  # Distance for constant velocity
        self.pos_ctrl_max = 1
        self.pos_ctrl_min = 0.0
        self.cmd_ctrl_max = 1 * cmd_ratio
        self.cmd_ctrl_min = -1 * cmd_ratio
        self.goal = None
        self.robot_pos = None
        self.robot_yaw = None

        self.track_cmd_ratio = 0.8

        rospy.loginfo("[%s] Initializing " % rospy.get_name())

        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.pos_control = PID_control("Position")
        self.ang_control = PID_control("Angular")

        robot = rospy.get_param("~robot", 'husky')
        if robot == 'locobot':
            self.pos_srv = Server(pos_loco_PIDConfig, self.pos_pid_cb, "Position")
            self.ang_srv = Server(ang_loco_PIDConfig, self.ang_pid_cb, "Angular")
        elif robot == 'jackal':
            self.pos_srv = Server(pos_jackal_PIDConfig, self.pos_pid_cb, "Position")
            self.ang_srv = Server(ang_jackal_PIDConfig, self.ang_pid_cb, "Angular")
        else:
            self.pos_srv = Server(pos_PIDConfig, self.pos_pid_cb, "Position")
            self.ang_srv = Server(ang_PIDConfig, self.ang_pid_cb, "Angular")

        self.initialize_PID()

        self.sub_goal = rospy.Subscriber(
            "pid_control/goal", PoseStamped, self.cb_goal, queue_size=1)

        # sub joy to switch on or off
        self.auto = True
        self.auto_robot = True
        self.sub_joy = rospy.Subscriber('joy_teleop/joy', Joy, self.cb_joy, queue_size=1)
        self.sub_go = rospy.Subscriber('robot_go', Bool, self.cb_go, queue_size=1)

        # tf v.s. Odometry => depend on user and scenario
        self.use_odom = rospy.get_param("~use_odom", False)
        self.use_tf = rospy.get_param("~use_tf", True)
        self.use_pose = rospy.get_param("~use_pose", False)

        if self.use_odom:
            self.sub_pose = rospy.Subscriber(
                'pid_control/pose', Odometry, self.cb_odom, queue_size=1)

        if self.use_tf:
            self.map_frame = rospy.get_param("~map_frame", 'map')
            self.robot_frame = rospy.get_param("~robot_frame", 'base_link')
            self.listener = tf.TransformListener()
        
        if self.use_pose:
            self.sub_pose = rospy.Subscriber(
                'robot_pose', PoseStamped, self.cb_pose, queue_size=1)

        self.cmd_msg = None
        self.acc = 0.25 # 1/(2/0.025)
        self.timer1 = rospy.Timer(rospy.Duration(0.1), self.ratio_adjust_loop)
        #self.timer2 = rospy.Timer(rospy.Duration(0.1), self.cmd_loop)

        # self.auto = True

    def cb_odom(self, msg):

        self.robot_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        quat = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        self.robot_yaw = yaw
    
    def cb_pose(self, msg):

        self.robot_pos = [msg.pose.position.x, msg.pose.position.y]
        quat = (msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        self.robot_yaw = yaw

    def control(self, goal_distance, goal_angle):
        self.pos_control.update(goal_distance)
        self.ang_control.update(goal_angle)

        # pos_output will always be positive
        pos_output = self.pos_constrain(-self.pos_control.output /
                                        self.dis4constV)

        # -1 = -180/180 < output/180 < 180/180 = 1
        ang_output = self.ang_control.output/180.
        return pos_output, ang_output

    def cb_goal(self, p):
        #rospy.logwarn("PID_control_cb_goal")
        self.goal = [p.pose.position.x, p.pose.position.y]

        if self.use_tf:
            try:
                (trans_c, rot_c) = self.listener.lookupTransform(
                    self.map_frame, self.robot_frame, rospy.Time(0))
                self.robot_pos = [trans_c[0], trans_c[1]]
                _, _, self.robot_yaw = tf.transformations.euler_from_quaternion(rot_c)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("faile to catch tf %s 2 %s" %
                            (self.map_frame, self.robot_frame))
                return

        if self.robot_pos is None:
            rospy.logwarn("%s : no robot pose" % rospy.get_name())
            return

        # self.robot_yaw = self.robot_yaw + np.pi/2
        goal_distance = self.get_distance(self.robot_pos, self.goal)
        goal_angle = self.get_goal_angle(
            self.robot_yaw, self.robot_pos, self.goal)

        pos_output, ang_output = self.control(goal_distance, goal_angle)

        #if self.auto:
        cmd_msg = Twist()
        cmd_msg.linear.x = self.cmd_constarin(pos_output)
        cmd_msg.angular.z = self.cmd_constarin(ang_output)
        self.cmd_msg = cmd_msg

    def cmd_loop(self, event):

        if self.cmd_msg is None:
            return
        cmd_msg = self.cmd_msg
        cmd_msg.linear.x = cmd_msg.linear.x * self.track_cmd_ratio
        cmd_msg.angular.z = cmd_msg.angular.z * self.track_cmd_ratio
        #self.pub_cmd.publish(cmd_msg)
    
    def ratio_adjust_loop(self, event):

        vel_mode = rospy.get_param("/velocity_mode", 4)

        if self.auto == False or self.auto_robot == False:
            vel_mode = 0
            return
        
        target_ratio = vel_mode * (1./7)

        if self.track_cmd_ratio < target_ratio:
            self.track_cmd_ratio += self.acc
            if self.track_cmd_ratio > target_ratio:
                self.track_cmd_ratio = target_ratio
        elif self.track_cmd_ratio > target_ratio:
            self.track_cmd_ratio -= self.acc
            if self.track_cmd_ratio < target_ratio:
                self.track_cmd_ratio = target_ratio
        if self.cmd_msg is None:
            return
        #self.track_cmd_ratio = 0.5
        # print("bbb", self.cmd_msg)
        cmd_msg = deepcopy(self.cmd_msg)
        cmd_msg.linear.x = cmd_msg.linear.x * target_ratio
        cmd_msg.angular.z = cmd_msg.angular.z * target_ratio
        self.pub_cmd.publish(cmd_msg)
        # print("before", self.cmd_msg)
        # print("after", cmd_msg)
        # print("ratio", self.track_cmd_ratio)

    def cmd_constarin(self, input):
        if input > self.cmd_ctrl_max:
            return self.cmd_ctrl_max
        if input < self.cmd_ctrl_min:
            return self.cmd_ctrl_min
        return input

    def pos_constrain(self, input):
        if input > self.pos_ctrl_max:
            return self.pos_ctrl_max
        if input < self.pos_ctrl_min:
            return self.pos_ctrl_min
        return input

    def initialize_PID(self):
        self.pos_control.setSampleTime(1)
        self.ang_control.setSampleTime(1)

        self.pos_control.SetPoint = 0.0
        self.ang_control.SetPoint = 0.0

    def get_goal_angle(self, robot_yaw, robot, goal):
        robot_angle = np.degrees(robot_yaw)
        p1 = [robot[0], robot[1]]
        p2 = [robot[0], robot[1]+1.]
        p3 = goal
        angle = self.get_angle(p1, p2, p3)
        result = angle - robot_angle
        result = self.angle_range(-(result + 90.))
        return result

    def get_angle(self, p1, p2, p3):
        v0 = np.array(p2) - np.array(p1)
        v1 = np.array(p3) - np.array(p1)
        angle = np.math.atan2(np.linalg.det([v0, v1]), np.dot(v0, v1))
        return np.degrees(angle)

    # limit the angle to the range of [-180, 180]
    def angle_range(self, angle):
        if angle > 180:
            angle = angle - 360
            angle = self.angle_range(angle)
        elif angle < -180:
            angle = angle + 360
            angle = self.angle_range(angle)
        return angle

    def get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def pos_pid_cb(self, config, level):
        print(
            "Position: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.pos_control.setKp(Kp)
        self.pos_control.setKi(Ki)
        self.pos_control.setKd(Kd)
        return config

    def ang_pid_cb(self, config, level):
        print(
            "Angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.ang_control.setKp(Kp)
        self.ang_control.setKi(Ki)
        self.ang_control.setKd(Kd)
        return config

    def cb_joy(self, joy_msg):
        # MODE D
        start_button = 7
        back_button = 6
        # Start button
        if (joy_msg.buttons[start_button] == 1) and not self.auto:
            self.auto = True
            rospy.loginfo('go auto')
        elif joy_msg.buttons[back_button] == 1 and self.auto:
            self.auto = False
            rospy.loginfo('go manual')
    
    def cb_go(self, msg):

        switch = msg.data
        # Start button
        if switch:
            self.auto_robot = True
        else:
            self.auto_robot = False


if __name__ == '__main__':
    rospy.init_node('pid_control')
    foo = Robot_PID()
    rospy.spin()
