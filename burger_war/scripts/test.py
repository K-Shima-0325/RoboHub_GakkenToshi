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
BURGER_RADIUS = 0.113 #[m]

#
FIELD_WIDTH_X = 1.697 #[m]
FIELD_WIDTH_Y = 1.697 #[m]

#Set Marker definition 
CENTER_BLOCK = 0
CORNER_BLOCK_1 = 1
CORNER_BLOCK_2 = 2
CORNER_BLOCK_3 = 3
CORNER_BLOCK_4 = 4

CENTER_BLOCK_WIDTH = 0.350
CENTER_BLOCK_RADIUS = 0.247

CORNER_BLOCK_WIDTH_X = 0.150
CORNER_BLOCK_WIDTH_Y = 0.200
CORNER_BLOCK_RADIUS = 0.125

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

MARKER_UP    = 1  #Blue side
MARKER_DOWN  = 2  #Red side
MARKER_RIGHT = 3
MARKER_LEFT  = 4

FIELD_AREA_BLUE_LU1 = 1 #Blue Left Up side 1
FIELD_AREA_BLUE_LL1 = 2
FIELD_AREA_RED_LL1  = 3
FIELD_AREA_RED_LD1  = 4
FIELD_AREA_RED_RD1  = 5 #Red Right Down side 1
FIELD_AREA_RED_RR1  = 6
FIELD_AREA_BLUE_RR1 = 7 
FIELD_AREA_BLUE_RU1 = 8
FIELD_AREA_BLUE_LU2 = 9 #Blue Left Up side 2
FIELD_AREA_BLUE_LL2 = 10
FIELD_AREA_RED_LL2  = 11
FIELD_AREA_RED_LD2  = 12
FIELD_AREA_RED_RD2  = 13 #Red Right Down side 2
FIELD_AREA_RED_RR2  = 14
FIELD_AREA_BLUE_RR2 = 15 
FIELD_AREA_BLUE_RU2 = 16
FIELD_OUTER_AREA = -1

#/Set Marker definition 

SIMPLE_GOAL_STATE_PENDING = 0
SIMPLE_GOAL_STATE_ACTIVE = 1
SIMPLE_GOAL_STATE_DONE = 2

ROTATE_CCW = 1
ROTATE_CW  = -1

FUNC_STATE_ERROR = -1
FUNC_STATE_ACTIVE = 0
FUNC_STATE_DONE = 1

BOT_MODE_STOP = 0
BOT_MODE_PATROL = 1
BOT_MODE_RUNAWAY = 2
BOT_MODE_HUNTING = 3

PATROL_ROTE_CCW = 1
PATROL_ROTE_CW  = -1

def JudgeFieldArea(x, y):
    if -CORNER_BLOCK_POS < x < CORNER_BLOCK_POS and -CORNER_BLOCK_POS < y < CORNER_BLOCK_POS:
        if x >= 0: 
            if y >= 0:
                if y <= x:
                    return FIELD_AREA_BLUE_LU1
                else: #y > x
                    return FIELD_AREA_BLUE_LL1
            else: # y < 0
                if y >= -x:
                    return FIELD_AREA_BLUE_RU1
                else: #y < -x
                    return FIELD_AREA_BLUE_RR1
        else: # x < 0
            if y >= 0:
                if y <= -x:
                    return FIELD_AREA_RED_LD1
                else: #y > -x
                    return FIELD_AREA_RED_LL1
            else: # y < 0
                if y >= x:
                    return FIELD_AREA_RED_RD1
                else: #y < x
                    return FIELD_AREA_RED_RR1 
    else:
        if x >= 0: 
            if y >= 0:
                if y <= x:
                    return FIELD_AREA_BLUE_LU2
                else: #y > x
                    return FIELD_AREA_BLUE_LL2
            else: # y < 0
                if y >= -x:
                    return FIELD_AREA_BLUE_RU2
                else: #y < -x
                    return FIELD_AREA_BLUE_RR2
        else: # x < 0
            if y >= 0:
                if y <= -x:
                    return FIELD_AREA_RED_LD2
                else: #y > -x
                    return FIELD_AREA_RED_LL2
            else: # y < 0
                if y >= x:
                    return FIELD_AREA_RED_RD2
                else: #y < x
                    return FIELD_AREA_RED_RR2     
   

class EnemyDetector:

    def __init__(self):
        self.max_distance = 1.2 # about 3.4 FIELD_WIDTH_X * 2 
        self.min_distance = BURGER_RADIUS     # about 0.11
        self.thresh_offset = 0.3
        self.thresh_corner = CORNER_BLOCK_WIDTH_X / 2 + BURGER_RADIUS + self.thresh_offset #about 0.188 + 
        self.thresh_center = CENTER_BLOCK_WIDTH / 2 + BURGER_RADIUS + self.thresh_offset   #about 0.288 +
        self.thresh_field = FIELD_WIDTH_Y - BURGER_RADIUS - self.thresh_offset             #about 1.584 -
        self.thresh_enemy_point = 5
        self.max_enemy_point = 30

        self.pose_x = 0
        self.pose_y = 0
        self.th = 0
        self.pre_th = 0
        self.thresh_th = 0.01746 #[rad] about 1.0 deg
    
        self.near_scan = 0
        self.pre_near_scan = 0
        self.thresh_d = 0.1 #[m]

    def findEnemy(self, scan, pose_x, pose_y, th):
        '''
        input scan. list of lider range, robot locate(pose_x, pose_y, th)
        return is_near_enemy(BOOL), enemy_direction[rad](float)
        '''
        if not len(scan) == 360:
            return False
        
        #for i, x in  enumerate(scan):
            #rospy.loginfo("Dir: {}, Dist: {}".format(i, x))

        # update pose
        self.pre_th = self.th
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.th = th

        # drop too big and small value ex) 0.0 , 2.0 
        self.pre_near_scan = self.near_scan
        self.near_scan = [x if self.max_distance > x > self.min_distance else 0.0 for x in scan]

        if -self.thresh_th < (self.th - self.pre_th) < self.thresh_th:
            for i in range(len(scan)):
                if -self.thresh_d < (self.near_scan[i] - self.pre_near_scan[i]) < self.thresh_d:
                    self.near_scan[i] = 0.0

        enemy_scan = [1 if self.is_point_emnemy(x,i) else 0 for i,x in enumerate(self.near_scan)]

        cnt_start = 0
        cnt_finish = 0
        enemy_candidate = 0

        for i in range(len(scan)):
            if   enemy_scan[0] == 1:
                for j in range(len(scan), 0):
                    if enemy_scan[j] == 1:
                        cnt_start -= 1
                    else:
                        break
            elif enemy_scan[i] == 1 and cnt_start == 0:
                cnt_start = i
            elif enemy_scan[i] == 0 and cnt_start != 0:
                cnt_finish = i
            elif enemy_scan[len(scan)-1] == 1 and cnt_start != 0:
                cnt_finish = i

            if cnt_finish != 0:
                if self.thresh_enemy_point < (cnt_finish - cnt_start) < self.max_enemy_point:
                    enemy_candidate += 1
                else:
                    if cnt_start < 0:
                        for j in range(len(scan), len(scan) + cnt_start):
                            enemy_scan[j] = 0
                        cnt_start = 0

                    for j in range(cnt_start, cnt_finish):
                        enemy_scan[j] = 0
                cnt_start = 0
            
            cnt_finish = 0

        if enemy_candidate == 1:
            is_near_enemy = True
        else:
            is_near_enemy = False
 
        if is_near_enemy:
            idx_l = [i for i, x in enumerate(enemy_scan) if x == 1]
            idx = idx_l[len(idx_l)/2]
            enemy_direction = idx / 360.0 * 2*PI
            enemy_dist = self.near_scan[idx]
            
            for i in range(len(idx_l)):
                 rospy.loginfo("idx: {}, Dir: {}[deg], Dist: {}[m]".format(i, idx_l[i], self.near_scan[idx_l[i]]))
                 rospy.loginfo("point_x: {}, point_y: {}".format(self.pose_x + self.near_scan[idx_l[i]] * math.cos(self.th + idx_l[i] / 360.0 * 2*PI), self.pose_y + self.near_scan[idx_l[i]] * math.sin(self.th + idx_l[i] / 360.0 * 2*PI)))
            rospy.loginfo("My_Pos_x: {}, My_Pos_y: {}".format(self.pose_x, self.pose_y))
            rospy.loginfo("My_Pos_dir: {}".format(self.th/2/PI*360))
            rospy.loginfo("Enemy_Dir: {}, Enemy_Dist: {}".format(enemy_direction, enemy_dist))
            rospy.loginfo("Enemy_x: {}, Enemy_y: {}".format(self.pose_x + enemy_dist * math.cos(self.th + enemy_direction), self.pose_y + enemy_dist * math.sin(self.th + enemy_direction)))
        else:
            enemy_direction = None
            enemy_dist = None

        rospy.loginfo("enemy points {}".format(sum(enemy_scan)))
        return is_near_enemy, enemy_direction, enemy_dist
        

    def is_point_emnemy(self, dist, ang_deg):
        if dist == 0:
            return False

        ang_rad = ang_deg /360. * 2 * PI
        point_x = self.pose_x + dist * math.cos(self.th + ang_rad)
        point_y = self.pose_y + dist * math.sin(self.th + ang_rad)

        #フィールド内かチェック
        if   point_y > (-point_x + self.thresh_field):
            return False
        elif point_y < (-point_x - self.thresh_field):
            return False
        elif point_y > ( point_x + self.thresh_field):
            return False
        elif point_y < ( point_x - self.thresh_field):
            return False

        #フィールド内の物体でないかチェック
        len_p1 = math.sqrt(pow((point_x - CORNER_BLOCK_1_POS_X), 2) + pow((point_y - CORNER_BLOCK_1_POS_Y), 2))
        len_p2 = math.sqrt(pow((point_x - CORNER_BLOCK_2_POS_X), 2) + pow((point_y - CORNER_BLOCK_2_POS_Y), 2))
        len_p3 = math.sqrt(pow((point_x - CORNER_BLOCK_3_POS_X), 2) + pow((point_y - CORNER_BLOCK_3_POS_Y), 2))
        len_p4 = math.sqrt(pow((point_x - CORNER_BLOCK_4_POS_X), 2) + pow((point_y - CORNER_BLOCK_4_POS_Y), 2))
        len_p5 = math.sqrt(pow(point_x  - CENTER_BLOCK_POS_X   , 2) + pow(point_y  - CENTER_BLOCK_POS_Y   , 2))

        if   len_p1 < self.thresh_corner:
            return False
        elif len_p2 < self.thresh_corner:
            return False
        elif len_p3 < self.thresh_corner:
            return False
        elif len_p4 < self.thresh_corner:
            return False
        elif len_p5 < self.thresh_center:
            return False
        else:
            #rospy.loginfo(point_x, point_y, self.pose_x, self.pose_y, self.th, dist, ang_deg, ang_rad)
            #rospy.loginfo(len_p1, len_p2, len_p3, len_p4, len_p5)
            return True
# End Respect

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
            self.scan_sec = 0

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

        #class EnemyDetector 
        self.enemy_detector = EnemyDetector()

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
        self.amcl_sec = 0

        #Bot position and direction (detrmined from all sensor data) 
        self.bot_pos_x = -1.0
        self.bot_pos_y = 0.0
        self.bot_dir = 0
        self.bot_pos_sec = 0
        self.bot_pre_pos_x = 0
        self.bot_pre_pos_y = 0
        self.bot_pre_dir = 0        
        self.bot_pre_pos_sec = -1.0
        self.is_initialized_pose = False
        self.is_bot_mode = BOT_MODE_STOP        

        #Ballon area on Camera data
        self.S = 0

        #Enemy detebt data
        self.is_near_enemy = False
        self.enemy_direction = None
        self.enemy_dist = None
    #/initialize parameter

    # lidar scan topic call back sample
    # update lidar scan state    
    def lidarCallback(self, data):
        self.scan = data
        scan_data = data.ranges
        self.scan_sec = data.header.stamp.to_sec()
        
        #rospy.loginfo("Scan Time: {}".format(self.scan_sec))
        #rospy.loginfo(self.scan)
        
        #Complement position 
        d_x   = self.bot_pos_x - self.bot_pre_pos_x
        d_y   = self.bot_pos_y - self.bot_pre_pos_y
        d_dir = self.bot_dir - self.bot_pre_dir
        if (self.scan_sec - self.bot_pos_sec) > 1.0:
            C_t = 0.5 / (self.bot_pos_sec - self.bot_pre_pos_sec)
        else: 
            C_t = (self.scan_sec - self.bot_pos_sec) / (self.bot_pos_sec - self.bot_pre_pos_sec)

        cmpl_pos_x   = self.bot_pos_x + d_x   * C_t
        cmpl_pos_y   = self.bot_pos_y + d_y   * C_t
        cmpl_pos_dir = self.bot_dir   + d_dir * C_t
        #rospy.loginfo("t1: {}, t2: {}, t3: {}".format(self.scan_sec, self.bot_pos_sec, self.bot_pre_pos_sec))

        # enemy detection
        if self.is_initialized_pose:
            self.is_near_enemy, self.enemy_direction, self.enemy_dist = self.enemy_detector.findEnemy(scan_data, cmpl_pos_x, cmpl_pos_y, cmpl_pos_dir)


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

    # amcl pose callback
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
        self.amcl_sec = data.header.stamp.to_sec()
        #rospy.loginfo("Pose Time: {}".format(self.amcl_sec))

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
        if self.bot_pos_sec != self.amcl_sec:
            self.bot_pre_pos_sec = self.bot_pos_sec
            self.bot_pre_pos_x = self.bot_pos_x
            self.bot_pre_pos_y = self.bot_pos_y
            self.bot_pre_dir = self.bot_dir        

            self.bot_pos_sec = self.amcl_sec
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

    def rotate(self, target_dir, rotate_dir = 0, rotate_vel = BURGER_MAX_ROTATE_V / 2, tolrerance_angle = (10*PI/180)):
        twist = Twist()
        max_loop = 10
        Decel_range = (30.0*PI/180.0)
                

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
        r = rospy.Rate(10) # change speed 10fps
        r.sleep()
        self.getBotPosition()
        self.init_x = self.bot_pos_x 
        self.init_y = self.bot_pos_y
        self.init_dir = self.bot_dir
        
        bot_start_sec = self.scan_sec
        bot_mode_stop_time = 0
        #rospy.loginfo("init_x: {}[m]".format(self.init_x))
        #rospy.loginfo("init_y: {}[m]".format(self.init_y))
        #rospy.loginfo("init_dir: {}[rad]".format(self.init_dir))

        route_state = 0
        self.goal_set_flag = 0
        active_flag = 0
        max_route_state = 30

        self.is_bot_mode = BOT_MODE_PATROL
        field_area = FIELD_AREA_RED_RD2
        patrol_rote = PATROL_ROTE_CW
        
        is_stopped = False
        dist_cnt = 0

        while not rospy.is_shutdown():

            if self.is_initialized_pose == False and (self.scan_sec - bot_start_sec) >= 20.0:
                self.is_initialized_pose = True

            if self.is_bot_mode == BOT_MODE_PATROL:

                #割込みでカメラ情報を読んで止まる
                if self.is_near_enemy == True or self.S > 2300:
                    self.stop()
                    if is_stopped == False:
                        is_stopped = True
                        bot_mode_stop_time = self.scan_sec

                    if self.enemy_direction != None: 
                        self.rotate(self.bot_dir + self.enemy_direction)
                
                    rospy.loginfo("--STOP MODE--")
                    if active_flag == 1 and self.goal_set_flag != 0:
                        self.client.cancel_all_goals()
                        self.goal_set_flag = 0

                    if self.enemy_dist < 0.4:
                        dist_cnt += 1

                    if (self.scan_sec - bot_mode_stop_time) > 12 or dist_cnt > 30:
                        bot_mode_stop_time = self.scan_sec
                        self.is_bot_mode = BOT_MODE_RUNAWAY
                        dist_cnt = 0
                        active_flag = 0
                        self.goal_set_flag = 0
                        self.client.cancel_all_goals()
                
                else:
                    is_stopped = False
                    if route_state == 0:
                        field_area = JudgeFieldArea(self.bot_pos_x, self.bot_pos_y)

                    if self.my_side != "b":
                        active_flag = self.Route_Strategy_2(field_area, route_state, patrol_rote)
                        #active_flag = self.Route_Strategy_1_redside(route_state)
                        #self.stop()
                    else:
                        active_flag = self.Route_Strategy_2(field_area, route_state, patrol_rote)
                        #active_flag = self.Route_Strategy_1_blueside(route_state)
                        #self.stop()
                    if active_flag == 0:
                        self.goal_set_flag = 0
                        route_state += 1
                        if route_state > max_route_state:
                            route_state = 0
                    elif active_flag < 0:
                        route_state = 0

            elif self.is_bot_mode == BOT_MODE_RUNAWAY:
                rospy.loginfo("--RUNAWAY MODE--") 
                twist = Twist()
                twist.linear.x = -BURGER_MAX_VELOCITY/3.0; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                self.vel_pub.publish(twist)
                if self.enemy_direction != None: 
                        self.rotate(self.bot_dir + self.enemy_direction)

                if (self.scan_sec - bot_mode_stop_time) > 2:
                    self.is_bot_mode = BOT_MODE_PATROL
                    patrol_rote *= -1
                    route_state = 4
                    

            #rospy.loginfo("odom pose_x: {}[m]".format(self.odom_pose_x))
            #rospy.loginfo("odom pose_y: {}[m]".format(self.odom_pose_y))
            #rospy.loginfo("odom orientation_yaw: {}[rad]".format(self.odom_orientation_yaw))
            #rospy.loginfo("imu orientation_yaw: {}[rad]".format(self.imu_orientation_yaw))
            #rospy.loginfo("route_state: {}".format(route_state))
            #rospy.loginfo("goal_set_flag: {}".format(self.goal_set_flag))
            #rospy.loginfo("active_flag: {}".format(active_flag))

        #self.setGoal(self.init_x,self.init_y,self.init_odom_yaw)    #

    def Route_Strategy_2(self, field, state, pat_rote):
        if   field == FIELD_AREA_RED_LD2 or field == FIELD_AREA_RED_RD2:        
            if pat_rote == PATROL_ROTE_CCW:
                if   state == 0:
                    return self.setGoal(-0.95, -0.2,-PI/4)
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
                else:
                    return -1
            else: #PATROL_ROTE_CW
                if   state == 0:
                    return self.setGoal(-0.95, 0.2, PI/4)
                elif state == 1:
                    return self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_DOWN, 0.2)
                elif state == 2:
                    return self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_DOWN)
                elif state == 3:
                    return self.setGoal(self.amcl_pose_x, self.amcl_pose_y,-PI/2)
                elif state == 4:
                    return self.setGoal(-0.53, 0, 0)
                elif state == 5:
                    return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_DOWN)
                elif state == 6:
                    return self.setGoal(0, 0.53, 0)
                else:
                    return -1
        elif field == FIELD_AREA_RED_LD1 or field == FIELD_AREA_RED_RD1:
            if pat_rote == PATROL_ROTE_CCW:
                if   state == 0:
                    return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_DOWN)
                elif state == 1:
                    return self.setGoal(0,-0.53, 0)
                else:
                    return -1
            else: #PATROL_ROTE_CW
                if   state == 0:
                    return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_DOWN)
                elif state == 1:
                    return self.setGoal(0, 0.53, 0)
                else:
                    return -1
        elif field == FIELD_AREA_RED_RR2 or field == FIELD_AREA_RED_RR1 or field == FIELD_AREA_BLUE_RR2 or field == FIELD_AREA_BLUE_RR1:
            if pat_rote == PATROL_ROTE_CCW:
                if state == 0:
                    return self.setGoal(0,-0.53, 0)
                elif state == 1:
                    return self.MoveToFieldMarker(CORNER_BLOCK_4, MARKER_DOWN)
                elif state == 2:
                    return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_RIGHT)
                elif state == 3:
                    return self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_UP)
                elif state == 4:
                    return self.setGoal(self.amcl_pose_x, self.amcl_pose_y,-PI/4)
                elif state == 5:
                    return self.setGoal(0.53-0.2,-0.53-0.5, 0)
                elif state == 6:
                    return self.setGoal(0.53+0.3,-0.53-0.05, PI/4)
                elif state == 7:
                    return self.MoveToFieldMarker(CORNER_BLOCK_4, MARKER_UP)
                else:
                    return -1
            else: #PATROL_ROTE_CW
                if state == 0:
                    return self.setGoal(0,-0.53, PI)
                elif state == 1:
                    return self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_UP)
                elif state == 2:
                    return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_RIGHT)
                elif state == 3:
                    return self.MoveToFieldMarker(CORNER_BLOCK_4, MARKER_DOWN)
                elif state == 4:
                    return self.setGoal(self.amcl_pose_x, self.amcl_pose_y, PI*3/4)
                elif state == 5:
                    return self.setGoal(-0.53+0.2, -0.53-0.5, PI)
                elif state == 6:
                    return self.setGoal(-0.53-0.3, -0.53-0.05, PI*3/4)
                elif state == 7:
                    return self.MoveToFieldMarker(CORNER_BLOCK_3, MARKER_DOWN)
                else:
                    return -1
        elif field == FIELD_AREA_BLUE_LU2 or field == FIELD_AREA_BLUE_RU2:        
            if pat_rote == PATROL_ROTE_CCW:
                if   state == 0:
                    return self.setGoal( 0.95, 0.2, PI*3/4)
                elif state == 1:
                    return self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_UP, 0.2)
                elif state == 2:
                    return self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_UP)
                elif state == 3:
                    return self.setGoal(self.amcl_pose_x, self.amcl_pose_y,-PI/2)
                elif state == 4:
                    return self.setGoal( 0.53, 0, 0)
                elif state == 5:
                    return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_UP)
                elif state == 6:
                    return self.setGoal(0, 0.53, 0)
                else:
                    return -1
            else: #PATROL_ROTE_CW
                if   state == 0:
                    return self.setGoal( 0.95, -0.2, -PI*3/4)
                elif state == 1:
                    return self.MoveToFieldMarker(CORNER_BLOCK_4, MARKER_UP, 0.2)
                elif state == 2:
                    return self.MoveToFieldMarker(CORNER_BLOCK_4, MARKER_UP)
                elif state == 3:
                    return self.setGoal(self.amcl_pose_x, self.amcl_pose_y, PI/2)
                elif state == 4:
                    return self.setGoal( 0.53, 0, 0)
                elif state == 5:
                    return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_UP)
                elif state == 6:
                    return self.setGoal(0, -0.53, 0)
                else:
                    return -1
        elif field == FIELD_AREA_BLUE_LU1 or field == FIELD_AREA_BLUE_RU1: 
            if pat_rote == PATROL_ROTE_CCW: 
                if   state == 0:
                    return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_UP)
                elif state == 1:
                    return self.setGoal(0, 0.53, 0)
                else:
                    return -1
            else: #PATROL_ROTE_CW
                if   state == 0:
                    return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_UP)
                elif state == 1:
                    return self.setGoal(0, -0.53, 0)
                else:
                    return -1
        elif field == FIELD_AREA_RED_LL2 or field == FIELD_AREA_RED_LL1 or field == FIELD_AREA_BLUE_LL2 or field == FIELD_AREA_BLUE_LL1:
            if pat_rote == PATROL_ROTE_CCW:
                if state == 0:
                    return self.setGoal(0, 0.53, PI)
                elif state == 1:
                    return self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_UP)
                elif state == 2:
                    return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_LEFT)
                elif state == 3:
                    return self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_DOWN)
                elif state == 4:
                    return self.setGoal(self.amcl_pose_x, self.amcl_pose_y,-PI*3/4)
                elif state == 5:
                    return self.setGoal(-0.53+0.2, 0.53+0.5, PI)
                elif state == 6:
                    return self.setGoal(-0.53-0.3, 0.53+0.05, -PI*3/4)
                elif state == 7:
                    return self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_DOWN)
                else:
                    return -1
            else: #PATROL_ROTE_CW
                if state == 0:
                    return self.setGoal(0, 0.53, 0)
                elif state == 1:
                    return self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_DOWN)
                elif state == 2:
                    return self.MoveToFieldMarker(CENTER_BLOCK, MARKER_LEFT)
                elif state == 3:
                    return self.MoveToFieldMarker(CORNER_BLOCK_2, MARKER_UP)
                elif state == 4:
                    return self.setGoal(self.amcl_pose_x, self.amcl_pose_y, PI/4)
                elif state == 5:
                    return self.setGoal(0.53-0.2, 0.53+0.5, 0)
                elif state == 6:
                    return self.setGoal(0.53+0.3, 0.53+0.05, -PI/4)
                elif state == 7:
                    return self.MoveToFieldMarker(CORNER_BLOCK_1, MARKER_UP)
                else:
                    return -1

if __name__ == '__main__':
    rospy.init_node('tofubot')
    bot = TofuBot('Tofu',True,True,True,True,True)
    bot.strategy()
