#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is TofuBot node.

by Fujiwara-Tofu.
'''

import rospy
import math
import numpy as np
import random

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

import re
#from aruco_msgs.msg import MarkerArray
import json
import requests
from cv_bridge import CvBridge, CvBridgeError
import cv2

import tf
#from tf.transformations import euler_from_quaternion

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

import sendIdToJudge

#
PI = math.pi

#Set TurtleBot3 Burger Specification
BURGER_MAX_VELOCITY = 0.22 #[m/s]
BURGER_MAX_ROTATE_V = 2.84 #[rad/s]

#Set Marker definition 
CENTER_BLOCK = 0
CORNER_BLOCK_1 = 1
CORNER_BLOCK_2 = 2
CORNER_BLOCK_3 = 3
CORNER_BLOCK_4 = 4

CENTER_BLOCK_WIDTH = 0.350
CORNER_BLOCK_WIDTH_X = 0.150
CORNER_BLOCK_WIDTH_Y = 0.200

CENTER_BLOCK_POS_X = 0
CENTER_BLOCK_POS_Y = 0

CORNER_BLOCK_POS = 0.530
CORNER_BLOCK_1_POS_X =  CORNER_BLOCK_POS
CORNER_BLOCK_1_POS_Y =  CORNER_BLOCK_POS
CORNER_BLOCK_2_POS_X = -CORNER_BLOCK_POS
CORNER_BLOCK_2_POS_Y =  CORNER_BLOCK_POS
CORNER_BLOCK_3_POS_X = -CORNER_BLOCK_POS
CORNER_BLOCK_3_POS_Y = -CORNER_BLOCK_POS
CORNER_BLOCK_4_POS_X =  CORNER_BLOCK_POS
CORNER_BLOCK_4_POS_Y = -CORNER_BLOCK_POS

MARKER_UP    = 1
MARKER_DOWN  = 2
MARKER_RIGHT = 3
MARKER_LEFT  = 4
#/Set Marker definition 

SIMPLE_GOAL_STATE_PENDING = 0
SIMPLE_GOAL_STATE_ACTIVE = 1
SIMPLE_GOAL_STATE_DONE = 2

ROTATE_CCW = 1
ROTATE_CW  = -1

FUNC_STATE_ERROR = -1
FUNC_STATE_ACTIVE = 0
FUNC_STATE_DONE = 1

class TargetInfo:
    def __init__(self, name, id, point):
        self.name = name
        self.player = "n"
        self.point = point


class TofuBot():
    def __init__(self, bot_name="NoName",use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):
        # bot name 
        self.name = bot_name

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

	# movebase client 
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)
            self.laser = self.scan.ranges

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

        # imu subscriber
        if use_imu:
            self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

        # odom subscriber
        if use_odom:
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # joint_states subscriber
        if use_joint_states:
            self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

	# amcl subscriber	
	self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)

        # set param from launch param
        self.judge_url = rospy.get_param('~judge_url', 'http://127.0.0.1:5000')

        '''
        # target ID  val subscriver
        self.target_id_sub = rospy.Subscriber('target_id', MarkerArray, self.targetIdCallback)
        '''

        # warstate subscriver
        self.war_state_sub = rospy.Subscriber('war_state', String, self.warstateCallback)
        self.warstate_json = 0 
        self.my_side = "n"

    #initialize parameter
        #odm
        self.odom_pose_x = 0
        self.odom_pose_y = 0
        self.odom_orientation_yaw = PI/2
        #imu
        self.imu_orientation_yaw = PI/2
        #amcl
        self.amcl_pose_x = 0
        self.amcl_pose_y = 0
        self.amcl_orientation_yaw = 0

        #Bot position and direction (detrmined from all sensor data) 
        self.bot_pos_x = 0
        self.bot_pos_y = 0
        self.bot_dir = 0
        #Ballon area on Camera data
        self.S = 0
    #/initialize parameter

    # lidar scan topic call back sample
    # update lidar scan state    
    def lidarCallback(self, data):
        self.scan = data
        #rospy.loginfo(self.scan)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.area(self.img) #
        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)


    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        #rospy.loginfo(data)
        self.imu_q = (data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
    
        imu_euler = tf.transformations.euler_from_quaternion(self.imu_q)
        self.imu_orientation_yaw = imu_euler[2]

        #rospy.loginfo("imu orientation_yaw: {}".format(self.imu_orientation_yaw))

    #odomCallback
    def odomCallback(self, data):
        self.odom_pose_x = data.pose.pose.position.x
        self.odom_pose_y = data.pose.pose.position.y

        odom_q = data.pose.pose.orientation
        odom_e = tf.transformations.euler_from_quaternion((odom_q.x,odom_q.y,odom_q.z,odom_q.w)) 
        self.odom_orientation_yaw = odom_e[2]
        
        #rospy.loginfo("odom pose_x: {}".format(self.odom_pose_x))
        #rospy.loginfo("odom pose_y: {}".format(self.odom_pose_y))
        #rospy.loginfo("odom orientation_yaw: {}".format(self.odom_orientation_yaw))
        
    #/odomCallback

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        #rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        #rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))


    def poseCallback(self, data):
        '''
        pose topic from amcl localizer
        update robot twist
        '''
        self.amcl_pose_x = data.pose.pose.position.x
        self.amcl_pose_y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))

        self.amcl_orientation_yaw = rpy[2]
        

    #--------------------------------------------------------------------#
    def warstateCallback(self, data):
        resp = requests.get(self.judge_url + "/warState")
        self.warstate_json = resp.json()
        #rospy.loginfo("------------------------")
        #rospy.loginfo("players : {}".format(self.warstate_json['players']))
        #rospy.loginfo("ready : {}".format(self.warstate_json['ready']))
        #rospy.loginfo("scores : {}".format(self.warstate_json['scores']))
        #rospy.loginfo("state : {}".format(self.warstate_json['state']))
        #for i in range(len(self.warstate_json['targets'])):
            #rospy.loginfo("targets {}: {}".format(i, self.warstate_json['targets'][i]))
        #rospy.loginfo("------------------------")

        if self.my_side == "n":
            if self.warstate_json['players']['r'] == 'you':
                #rospy.loginfo("red players : {}".format(self.warstate_json['players']['r']))
                self.my_side = "r"
            elif self.warstate_json['players']['b'] == 'you':
                #rospy.loginfo("blue players : {}".format(self.warstate_json['players']['b']))
                self.my_side = "b"
            else:
                self.my_side = "n"        
        #rospy.loginfo("my side : {}".format(self.my_side))


    '''
    def getInfoFromJudge(self):
        #rospy.loginfo("Judge URL: {}".format(self.judge_url))
        return 0
    '''
    #--------------------------------------------------------------------#
    #---- Get tareget ID (Ref. sendIdToJudge) ---------------------------------------------#
    '''
    def lengthTo4(self, string):
        length = len(string)
        if length == 4:
            return string
        elif length > 4:
            return string[-4:]
        elif length < 4:
            return ("0000"+string)[-4:]
        else:
            rospy.logerr("Unexpected error in lendthTo4()")
            return False

    def targetIdCallback(self, data):
        markers = data.markers
        for marker in markers:
            target_id = self.lengthTo4(str(marker.id))
            rospy.loginfo("target_id: {}".format(target_id))
    '''
    #---- \Get tareget ID (Ref. sendIdToJudge) ---------------------------------------------#
    
    #--------------------------------------------------------------------#
    def area(self,img):
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        hsvLower = np.array([0, 128, 0])
        hsvUpper = np.array([30, 255, 255])
        mask1 = cv2.inRange(hsv, hsvLower, hsvUpper)

        hsvLower = np.array([150, 128, 0])
        hsvUpper = np.array([179, 255, 255])
        mask2 = cv2.inRange(hsv, hsvLower, hsvUpper)
    
        mask = mask1 + mask2

        masked_hsv = cv2.bitwise_and(img, img, mask=mask)
        gray = cv2.cvtColor(masked_hsv,cv2.COLOR_BGR2GRAY)

        ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY)
        image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            self.S = cv2.contourArea(contours[0])
            #rospy.logerr("Count area : Success, S={}".format(self.S))
            #cv2.imshow("Enermy marker snap", image)
        else:
            self.S = 0
            #rospy.logerr("Count area : Failed, S={}".format(self.S))
        return 0            
    #--------------------------------------------------------------------#
    def getBotPosition(self):
        self.bot_pos_x = self.amcl_pose_x 
        self.bot_pos_y = self.amcl_pose_y
        angle = self.amcl_orientation_yaw
        max_loop = 10
        for i in range(max_loop):
            if (-PI <= angle and angle < PI):
                break
            elif angle < -PI:
                angle += 2*PI
            elif angle >= PI:
                angle -= 2*PI
            else:
                rospy.logerr("angle calculation failure")
        else:
            rospy.logerr("angle calculation loop reach to max : {}".format(angle))
        self.bot_dir = angle        

    def stop(self):
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.vel_pub.publish(twist)

    def rotate(self, target_dir, rotate_dir = 0, rotate_vel = BURGER_MAX_ROTATE_V, tolrerance_angle = (10*PI/180)):
        twist = Twist()
        max_loop = 10
        Decel_range = (20*PI/180)
        
        self.getBotPosition()
        diff_angle = target_dir - self.bot_dir

        for i in range(max_loop):
            if (-PI <= diff_angle and diff_angle < PI):
                break
            elif diff_angle < -PI:
                diff_angle += 2*PI
            elif diff_angle >= PI:
                diff_angle -= 2*PI
            else:
                rospy.logerr("diff_angle calculation failure")
        else:
            rospy.logerr("diff_angle calculation loop reach to max : {}".format(diff_angle))
            diff_angle = 0
            return FUNC_STATE_ERROR

	if (np.abs(diff_angle) < np.abs(tolrerance_angle)):            
            twist.angular.z = 0
            self.vel_pub.publish(twist)
            return FUNC_STATE_DONE

        if rotate_dir == ROTATE_CCW: 
	    twist.angular.z = rotate_vel	#CCW
        elif rotate_dir == ROTATE_CW:
	    twist.angular.z = -rotate_vel	#CW
	else:
            if diff_angle >= 0: 
                twist.angular.z = rotate_vel	#CCW
            else:		
                twist.angular.z = -rotate_vel	#CW

	if (np.abs(diff_angle) < np.abs(Decel_range)):         
            twist.angular.z *= np.abs(diff_angle / Decel_range)

        self.vel_pub.publish(twist)
        return FUNC_STATE_ACTIVE


    def judgeGoal(self,goal_x,goal_y,tolerance_r = 0.1):
        self.getBotPosition()
        if ((tolerance_r)**2) > ((goal_x - self.bot_pos_x)**2 + (goal_y - self.bot_pos_y)**2):
            return True        
        else:
            return False

    #---- Move to goal by MoveBaseGoal() ---------------------------------------------#
    def setGoal(self,x,y,yaw, judegGaoal_flag = True):
        if self.goal_set_flag == 0:
            self.client.wait_for_server()
    
            #Coordinate transformation (Odom -> Move Base)
            movebase_x = x
            movebase_y = y
            movebase_yaw = yaw
            #/Coordinate transformation (Odom -> Move Base)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "/map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = movebase_x
            goal.target_pose.pose.position.y = movebase_y

            # Euler to Quartanion
            q=tf.transformations.quaternion_from_euler(0,0,movebase_yaw)        
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]

            self.client.send_goal(goal)
            self.goal_set_flag = 1

        if judegGaoal_flag == True:
            if self.judgeGoal(x,y) == True:
                self.client.cancel_all_goals()
                #rospy.loginfo("Reach to goal around")
                if self.rotate(yaw) == 1:
                    #rospy.loginfo("Turn goal direction")
                    return 0
                else:
                    return 1

        if not self.client.simple_state == SIMPLE_GOAL_STATE_DONE:
            if not self.client.gh:
                rospy.logerr("Called wait_for_goal_to_finish when no goal exists")
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                return -1
            else:
                return 1
        else:        
            self.client.get_result()
            return 0
    #---- \Move to goal by by MoveBaseGoal() ---------------------------------------------#      

    #---- Move to Field malker by setGoal() ---------------------------------------------#
    def MoveToFieldMarker(self,block_name, marker_name, shooting_distance = 0.300):
        if self.goal_set_flag == False:
            if block_name == CENTER_BLOCK:
                if marker_name == MARKER_UP:
                    self.goal_pos_x = CENTER_BLOCK_POS_X + (CENTER_BLOCK_WIDTH/2 + shooting_distance)
                    self.goal_pos_y = CENTER_BLOCK_POS_Y
                    self.goal_angle = -PI       
                elif marker_name == MARKER_DOWN:
                    self.goal_pos_x = CENTER_BLOCK_POS_X - (CENTER_BLOCK_WIDTH/2 + shooting_distance)
                    self.goal_pos_y = CENTER_BLOCK_POS_Y
                    self.goal_angle = 0    
                elif marker_name == MARKER_RIGHT:
                    self.goal_pos_x = CENTER_BLOCK_POS_X
                    self.goal_pos_y = CENTER_BLOCK_POS_Y - (CENTER_BLOCK_WIDTH/2 + shooting_distance)
                    self.goal_angle = PI/2
                elif marker_name == MARKER_LEFT:
                    self.goal_pos_x = CENTER_BLOCK_POS_X
                    self.goal_pos_y = CENTER_BLOCK_POS_Y + (CENTER_BLOCK_WIDTH/2 + shooting_distance)   
                    self.goal_angle = -PI/2
                else:
                    rospy.logerr("Invalid_MARKER_NAME")
                    return -1
    
            elif block_name == CORNER_BLOCK_1:
                if marker_name == MARKER_UP:
                    self.goal_pos_x = CORNER_BLOCK_1_POS_X + (CORNER_BLOCK_WIDTH_X/2 + shooting_distance)
                    self.goal_pos_y = CORNER_BLOCK_1_POS_Y 
                    self.goal_angle = -PI           
                elif marker_name == MARKER_DOWN:
                    self.goal_pos_x = CORNER_BLOCK_1_POS_X - (CORNER_BLOCK_WIDTH_X/2 + shooting_distance)
                    self.goal_pos_y = CORNER_BLOCK_1_POS_Y
                    self.goal_angle = 0        
                else:
                    rospy.logerr("Invalid_MARKER_NAME")
                    return -1

            elif block_name == CORNER_BLOCK_2:
                if marker_name == MARKER_UP:    
                    self.goal_pos_x = CORNER_BLOCK_2_POS_X + (CORNER_BLOCK_WIDTH_X/2 + shooting_distance)  
                    self.goal_pos_y = CORNER_BLOCK_2_POS_Y
                    self.goal_angle = -PI      
                elif marker_name == MARKER_DOWN:
                    self.goal_pos_x = CORNER_BLOCK_2_POS_X - (CORNER_BLOCK_WIDTH_X/2 + shooting_distance)
                    self.goal_pos_y = CORNER_BLOCK_2_POS_Y
                    self.goal_angle = 0 
                else:
                    rospy.logerr("Invalid_MARKER_NAME")
                    return -1

            elif block_name == CORNER_BLOCK_3:
                if marker_name == MARKER_UP:
                    self.goal_pos_x = CORNER_BLOCK_3_POS_X + (CORNER_BLOCK_WIDTH_X/2 + shooting_distance)
                    self.goal_pos_y = CORNER_BLOCK_3_POS_Y
                    self.goal_angle = -PI         
                elif marker_name == MARKER_DOWN:
                    self.goal_pos_x = CORNER_BLOCK_3_POS_X - (CORNER_BLOCK_WIDTH_X/2 + shooting_distance)    
                    self.goal_pos_y = CORNER_BLOCK_3_POS_Y
                    self.goal_angle = 0
                else:
                    rospy.logerr("Invalid_MARKER_NAME")
                    return -1

            elif block_name == CORNER_BLOCK_4:
                if marker_name == MARKER_UP:    
                    self.goal_pos_x = CORNER_BLOCK_4_POS_X + (CORNER_BLOCK_WIDTH_X/2 + shooting_distance)
                    self.goal_pos_y = CORNER_BLOCK_4_POS_Y
                    self.goal_angle = -PI       
                elif marker_name == MARKER_DOWN:
                    self.goal_pos_x = CORNER_BLOCK_4_POS_X - (CORNER_BLOCK_WIDTH_X/2 + shooting_distance)
                    self.goal_pos_y = CORNER_BLOCK_4_POS_Y
                    self.goal_angle = 0   
                else:
                    rospy.logerr("Invalid_MARKER_NAME")
                    return -1
        
            else:
                rospy.logerr("Invalid_MARKER_NAME")
                return -1
        
        return self.setGoal(self.goal_pos_x,self.goal_pos_y,self.goal_angle)
    #---- \Move to Field malker by setGoal() ---------------------------------------------#

    
    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        r.sleep()
        self.init_x = self.amcl_pose_x
        self.init_y = self.amcl_pose_y
        #self.init_odom_yaw = self.odom_orientation_yaw
        #self.init_imu_yaw = self.imu_orientation_yaw
        #rospy.loginfo("init_x: {}[m]".format(self.init_x))
        #rospy.loginfo("init_y: {}[m]".format(self.init_y))
        #rospy.loginfo("init_yaw(odom): {}[rad]".format(self.init_odom_yaw))
        #rospy.loginfo("init_yaw(imu): {}[rad]".format(self.init_imu_yaw))
       
        route_state = 0
        self.goal_set_flag = 0
        active_flag = 0
        max_route_state = 30
        while not rospy.is_shutdown():
            #割込みでカメラ情報を読んで止まる
            if self.S > 2000:
                #twist = self.stop()
                rospy.loginfo("--STOP Moving! Enemy Ahead!--")
                #if active_flag == 1:
                    #self.client.cancel_all_goals()
                    #self.goal_set_flag = 0

            else:
                if self.my_side != "b":
                    active_flag = self.Rotue_Strategy_1_redside(route_state)
                else:
                    active_flag = self.Rotue_Strategy_1_blueside(route_state)
                
                if active_flag == 0:
                    self.goal_set_flag = 0
                    route_state += 1
                    if route_state > max_route_state:
                        route_state = 0

            #rospy.loginfo("odom pose_x: {}[m]".format(self.odom_pose_x))
            #rospy.loginfo("odom pose_y: {}[m]".format(self.odom_pose_y))
            #rospy.loginfo("odom orientation_yaw: {}[rad]".format(self.odom_orientation_yaw))
            #rospy.loginfo("imu orientation_yaw: {}[rad]".format(self.imu_orientation_yaw))
            #rospy.loginfo("route_state: {}".format(route_state))
            #rospy.loginfo("goal_set_flag: {}".format(self.goal_set_flag))
            #rospy.loginfo("active_flag: {}".format(active_flag))

        #self.setGoal(self.init_x,self.init_y,self.init_odom_yaw)    #

    def Rotue_Strategy_1_redside(self, state):        
        if state == 0:
            return self.setGoal(-0.9,-0.2,-PI/4) 
            #return self.setGoal(0,0.9,PI/2) #for test
        elif state == 1:
            return self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_DOWN, 0.2)
        elif state == 2:
            return self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_DOWN)
        elif state == 3:
            return self.setGoal(self.amcl_pose_x, self.amcl_pose_y,PI/2)
        elif state == 4:
            return self.setGoal(-0.53, 0, 0)
        elif state == 5:
            return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_DOWN)
        elif state == 6:
            return self.setGoal(0,-0.53, 0)
        elif state == 7:        
            return self.MoveToFieldMarker(CORNER_BLOCK_4, MARKER_DOWN)
        elif state == 8:
            return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_RIGHT)
        elif state == 9:
            return self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_UP)
        elif state == 10:
            return self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_UP)
        elif state == 11:
            return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_LEFT)
        elif state == 12:
            return self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_DOWN)
        elif state == 13:
            return self.setGoal(0,0.53,PI)
        elif state == 14:    
            return self.setGoal(-0.53, 0,PI)
        elif state == 15:
            return self.setGoal(-0.90, 0,PI)
        elif state == 16:
            return self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_DOWN, 0.2)
        elif state == 17:
            return self.setGoal(self.amcl_pose_x, self.amcl_pose_y,-PI/2)
        else:
            return 0

    def Rotue_Strategy_1_blueside(self, state):        
        if state == 0:
            return self.setGoal(0.9,0.2,3*PI/4) 
        elif state == 1:
            return self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_UP, 0.2)
        elif state == 2:
            return self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_DOWN)
        elif state == 3:
            return self.setGoal(self.amcl_pose_x, self.amcl_pose_y,-PI/2)
        elif state == 4:
            return self.setGoal(0.53, 0, PI)
        elif state == 5:
            return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_UP)
        elif state == 6:
            return self.setGoal(0, 0.53, PI)
        elif state == 7:        
            return self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_UP)
        elif state == 8:
            return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_LEFT)
        elif state == 9:
            return self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_DOWN)
        elif state == 10:
            return self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_DOWN)
        elif state == 11:
            return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_RIGHT)
        elif state == 12:
            return self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_UP)
        elif state == 13:
            return self.setGoal(0,-0.53, 0)
        elif state == 14:    
            return self.setGoal(0.53, 0, 0)
        elif state == 15:
            return self.setGoal(0.90, 0, 0)
        elif state == 16:
            return self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_UP, 0.2)
        elif state == 17:
            return self.setGoal(self.amcl_pose_x, self.amcl_pose_y,PI/2)
        else:
            return 0

    def Rotue_Strategy_2_redside(self, state):        
        if state == 0:
            return self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_DOWN)
        elif state == 1:
            return self.setGoal(CORNER_BLOCK_3_POS_X, CORNER_BLOCK_3_POS_Y - 0.25, 0)
        elif state == 2:
            return self.MoveToFieldMarker(CORNER_BLOCK_4, MARKER_DOWN)
        elif state == 3:
            return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_RIGHT)
        elif state == 4:
            return self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_UP)
        elif state == 5:
            return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_DOWN)
        elif state == 6:
            return self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_DOWN)
        elif state == 7:        
            return self.setGoal(CORNER_BLOCK_2_POS_X, CORNER_BLOCK_2_POS_Y + 0.25, 0)
        elif state == 8:
            return self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_UP)
        elif state == 9:
            return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_LEFT)
        elif state == 10:
            return self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_DOWN)
        elif state == 11:
            return self.setGoal(CORNER_BLOCK_1_POS_X, CORNER_BLOCK_1_POS_Y + 0.25, 0)
        elif state == 12:
            return self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_UP)
        elif state == 13:
            return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_UP)
        elif state == 14:    
            return self.MoveToFieldMarker(CORNER_BLOCK_4, MARKER_UP)
        elif state == 15:
            return self.setGoal(self.amcl_pose_x, self.amcl_pose_y,-PI/2)
        else:
            return 0


if __name__ == '__main__':
    rospy.init_node('tofubot')
    bot = TofuBot('Tofu',True,True,True,True,True)
    bot.strategy()
