#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from rise_control.msg import ppm_msg
import numpy as np
import time
import math
import numpy.linalg as lin
import tf

class control:
    def calculating_rpy(self,q0,q1,q2,q3):
        # calculating quaternion -> roll pitch yaw
        roll = math.atan2(2*(q0*q1 + q2*q3),(1-2*(q1**2 + q2**2)))
        pitch = math.asin(2*(q0*q2 - q3*q1))
        yaw = math.atan2(2*(q0*q3 + q1*q2),1-2*(q2**2 + q3**2))
        return roll, pitch, yaw

    def motion_cb(self, msg):
        self.mot_msg = msg
        self.motion_time = self.mot_msg.header.stamp.secs + self.mot_msg.header.stamp.nsecs * 10 ** (-9)
        self.motion_quat_x = self.mot_msg.pose.orientation.x
        self.motion_quat_y = self.mot_msg.pose.orientation.y
        self.motion_quat_z = self.mot_msg.pose.orientation.z
        self.motion_quat_w = self.mot_msg.pose.orientation.w
        self.motion_x = self.mot_msg.pose.position.x
        self.motion_y = self.mot_msg.pose.position.y
        self.motion_z = self.mot_msg.pose.position.z

    def ppm_cb(self, msg):
        self.ppm_input_msg = msg
        self.ch1 = int(self.ppm_input_msg.channel_1)
        self.ch2 = int(self.ppm_input_msg.channel_2)
        self.ch3 = int(self.ppm_input_msg.channel_3)
        self.ch4 = int(self.ppm_input_msg.channel_4)
        self.ch5 = int(self.ppm_input_msg.channel_5)
        self.ch6 = int(self.ppm_input_msg.channel_6)
        self.ch7 = int(self.ppm_input_msg.channel_7)
        self.ch8 = int(self.ppm_input_msg.channel_8)

    def __init__(self, x, y ,z):
        # reference point
        self.target_x = x
        self.target_y = y
        self.target_z = z

        self.ch1 = 1500
        self.ch2 = 1000
        self.ch3 = 1000
        self.ch4 = 1000
        self.ch5 = 1000
        self.ch6 = 1500
        self.ch7 = 1000
        self.ch8 = 1000
        self.motion_x = 0.0
        self.motion_y = 0.0
        self.motion_z = 0.0
        self.motion_quat_x = 0.0
        self.motion_quat_y = 0.0
        self.motion_quat_z = 0.0
        self.motion_quat_w = 0.0
        self.motion_time = 0.0
        self.motion_time_prev = 0.0
        self.motion_roll = 0.0
        self.motion_pitch = 0.0
        self.motion_yaw = 0.0
        # Subscriber created
        rospy.Subscriber("/vrpn_client_node/quad_imu_2/pose", PoseStamped, self.motion_cb)
        rospy.Subscriber("/input_ppm", ppm_msg, self.ppm_cb)
        self.controling_pub = rospy.Publisher("/control_signal", ppm_msg, queue_size=1)

    def pid(self):


