#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import tf2_ros
import numpy as np
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion
import tf.transformations 
from std_srvs.srv import Trigger, TriggerResponse
from geographic_msgs.msg import GeoPoint
from geodesy import utm
from geometry_msgs.msg import PoseStamped, Twist, PoseArray,PointStamped,Quaternion
from robot_control.msg import controlData 
from robot_localization.msg import INSPVAE,INSPVA,baseStatus, GPSData
from std_msgs.msg import Int16, Int32, Header, String
from sensor_msgs.msg import Image
import time
import copy
import json
class ArucoDockingController:
    def __init__(self):
        
        self.info = True
        self.logprint=False
        # 坐标系参数
        self.marker_spacing = rospy.get_param('~marker_spacing', 1.0)  # 左右标记间距（米）
        self.marker_side_spacing   = rospy.get_param('~marker_side_spacing', 0.78)  # 中间标记与侧标记间距（米）
        self.stop_distance = rospy.get_param('~stop_distance', 0.7)  # 中间标记前停止距离
        self.stop_distance_threshold = rospy.get_param('stop_distance_threshold', 0.1)  # 停止距离阈值
        self.angle_dir = rospy.get_param('~angle_dir', 1)  # 角度方向（1表示顺时针，-1表示逆时针）
        self.target_distance = 1 # 目标距离（米）
        self.stop_refine_pose_dlt_y=0.06
        self.align_threshold = math.radians(1)  # 航向对准阈值
        self.current_yaw = 0 # 当前航向角
        self.target_yaw = 0# 目标航向角
        # self.latitude = 30.32101833   
        # self.longitude = 120.07105   
        self.latitude = 30.32098152262    #test
        self.longitude = 120.07004748195  #test

        self.latitude_drone = 30.32098151262
        self.longitude_drone = 120.07004749195
        self.gps_yaw = 0.0
        self.yaw_drone = 0.0
        self.speed = 0.0
        self.distance2drone = 0.0
        self.yaw2drone = 0.0
        self.depth_image = None
        self.back=False
        self.search_count = 0
        self.lock_current=False
        self.lock_refine=False
        self.align_num=False        # False  调试control_loop粗对正时暂为True
        self.refine_align=False     # 调试精确对正标志位 调试位True
        self.rc_control = 1         # 遥控器控制状态 默认0，调试control_loop时暂为1
        self.complete_state = 2     #默认0 , 调试control_loop时为2


        # TF配置
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.flag_count = 0
        # 新增数据有效期参数（单位：秒）
        self.data_expiry = 0.6  # 0.5秒未更新的数据视为失效
        self.marker_time = {'left': None, 'right': None, 'center': None, 'center_left': None, 'center_right': None}
        self.valid_center_markers = []
        self.center_side_offset = [ 0.37608,-0.01905,-0.42418]
        self.first_look_flag = False
        self.count = 0

        # 存储检测数据（基坐标系）
        self.markers = {
            'left': None, 
            'right': None,
            'center': None,
            'leftside':None,
            'rightside':None
        }
        # self.depth_dict={}
        self.markers_orientation = {}


        # 状态变量
        self.state = "INIT"
        self.state_prev = "INIT"
        self.estimated_center = None
        self.current_target = {
            'position': np.array([0.0, 0.0, 0.0]),
            'yaw': 0.0,
            'center': np.array([0.0, 0.0, 0.0]),
        }
        self.side_target = {
            'position': np.array([0.0, 0.0, 0.0]),
            'yaw': 0.0,
            'center': np.array([0.0, 0.0, 0.0]),
        }


        self.control_seq = 0
        self.stop_flag = False
        self.out_dock_flag = False
        self.in_dock_flag = True
        self.corner_finding_flag = True
        self.auto_cleaning_flag = True
        self.docking_flag = False
     
        # 订阅器
        rospy.Subscriber("/inspvae_data", INSPVAE, self.inspvae_cb)
        rospy.Subscriber("/inspva_data", INSPVA, self.inspva_cb)
        rospy.Subscriber("/base_status", baseStatus, self.base_cb)
        rospy.Subscriber("/gps/raw", String, self.drone_gps_cb)

        rospy.Subscriber("/camera/aruco_102/pose", PoseStamped, self.leftedge_cb)
        rospy.Subscriber("/camera/aruco_103/pose", PoseStamped, self.rightedge_cb)
        rospy.Subscriber("/camera/aruco_104/pose", PoseStamped, self.center_cb)
        rospy.Subscriber("/camera/aruco_100/pose", PoseStamped, self.leftside_cb)
        rospy.Subscriber("/camera/aruco_101/pose", PoseStamped, self.rightside_cb)

        
        # 发布器
        self.control_pub = rospy.Publisher("/control_data", controlData, queue_size=1)

        self.target_pub = rospy.Publisher("/virsual_1", PoseStamped, queue_size=1)


        # rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.Timer(rospy.Duration(0.1), self.control_loop_test)

#------------------------------------CALLBACK---------------------------------------------------------------------------------------------------
   
    def base_cb(self, msg):
        """处理基坐标系状态数据"""
        # 处理IMU数据
        self.speed = msg.speed
        self.distance_base = msg.distance
        self.sensor_state = msg.sensor_state
        self.complete_state = msg.complete_state
        self.rc_control = msg.rc_state
        # self.battery = msg.battery # 电池电量(todo)

    def drone_gps_cb(self, msg):
        """处理无人机GPS数据"""
        try:
            # 将JSON格式的字符串解析为Python字典
            gps_data = json.loads(msg.data)
            
            # 输出GPS数据为8位浮点数
            # 从gps_data中提取latitude和longitude
            latitude_str = gps_data.get("latitude", "N/A")
            longitude_str = gps_data.get("longitude", "N/A")

            # 确保将其转换为浮动类型，如果是有效值（不是"N/A"），否则使用默认值0.0
            self.latitude_drone = float(latitude_str) if latitude_str != "N/A" else 0.0
            self.longitude_drone = float(longitude_str) if longitude_str != "N/A" else 0.0
            self.yaw_drone = float(gps_data.get("heading", "N/A")) if gps_data.get("heading", "N/A") != "N/A" else 0.0
            # 保留8位小数，确保数据格式正确
            self.latitude_drone = round(self.latitude_drone, 8)
            self.longitude_drone = round(self.longitude_drone, 8)
            self.yaw_drone = math.radians(round(self.yaw_drone, 8))  # 转换为弧度
            # 航向 ( 0 到 360.0 ) 航向是主天线至从天线方向间基线向量逆时针方向与真北的夹角
            # 输出保留8位小数的经纬度
            rospy.loginfo("Latitude: %.8f, Longitude: %.8f, Yaw_drone: %.8f", self.latitude_drone, self.longitude_drone, self.yaw_drone)

        except json.JSONDecodeError:
            rospy.logerr("Received invalid JSON data")
        # self.latitude_drone = msg.latitude
        # self.longitude_drone = msg.longitude
        # self.yaw_drone = msg.yaw

    def inspvae_cb(self, msg):
        # self.latitude = msg.latitude
        # self.longitude = msg.longitude
        self.current_yaw = math.radians(msg.yaw)

    def inspva_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.gps_yaw = math.radians(msg.yaw)
        

 
    def leftedge_cb(self, msg):
        self.pose_callback(msg,"left")

    def rightedge_cb(self, msg):
        self.pose_callback(msg,"right")

    def center_cb(self, msg):
        self.pose_callback(msg,"center")

    def leftside_cb(self, msg):
        self.pose_callback(msg,"leftside")

    def rightside_cb(self, msg):
        self.pose_callback(msg,"rightside")

    def update_state(self):
        """状态机更新（增加数据有效性检查）"""
        self.check_data_expiry()  # 先执行数据清理
        self.valid_center_markers = []
        valid_target = []
        left_right=[]
        current_target = {
            'position': np.array([0.0, 0.0, 0.0]),
            'yaw': 0.0,
            'center': np.array([0.0, 0.0, 0.0]),
        }
        side_target = {
            'position': np.array([0.0, 0.0, 0.0]),
            'yaw': 0.0,
            'center': np.array([0.0, 0.0, 0.0]),
        }

        if self.markers['center'] is not None: 
            # self.state = "FINAL_APPROACH"
            
            self.valid_center_markers.append(self.markers['center'])

            ct1=self.calculate_center_target()
            if ct1 is not None:
                valid_target.append(ct1)
 
        if self.markers['left'] is not None:
            left_target = self.test_center_side_target('left')  
            if left_target is not None: 
                valid_target.append(left_target)
                left_right.append(left_target)
                # rospy.loginfo(f"valid_target: {valid_target}")

        
        if self.markers['right'] is not None:
            right_target = self.test_center_side_target('right')    
            if right_target is not None:    
                valid_target.append(right_target)   
                left_right.append(right_target)
        
        if self.markers['leftside'] is not None:
            side_target = self.calculate_center_front_target('leftside')
            if side_target is not None:
                self.side_target = side_target

        if self.markers['rightside'] is not None:
            side_target = self.calculate_center_front_target('rightside')
            if side_target is not None:
                self.side_target = side_target


        if self.markers['left'] or self.markers['right'] or self.markers['center']:
            self.state = "APPROACHING"
            if self.first_look_flag == False:
                self.first_look_flag = True
               
                time.sleep(0.1)
                    
        else:
            self.state = "SEARCH"
            # rospy.loginfo(f"update.state: {self.state}")
            self.first_look_flag = False
            if self.lock_current==False:
                self.current_target = None  # 清空目标


        if len(valid_target)>0:
            for target in valid_target:
                # rospy.loginfo(f"target: {target}")
                current_target['position']+= target['position']     
                current_target['yaw']+= target['yaw']
                current_target['center']+= target['center']  
                    
            current_target['position'] /= len(valid_target)
            current_target['yaw'] /= len(valid_target)
            current_target['center'] /= len(valid_target)  

            self.current_target = current_target   
            # rospy.loginfo(f"current_target: {self.current_target}") 
                        
            return      
    
    
    
    def pose_callback(self,msg,marker_type):
        try:
            self.lock_current = True  #??
            self.markers[marker_type] = msg.pose.position
            self.marker_time[marker_type] = msg.header.stamp # 记录时间戳
            # self.markers_pixel[marker_type] = msg.pose.pixel
            # self.depth_dict[marker_type] =msg.pose.position.z
            self.markers_orientation[marker_type] = msg.pose.orientation
            #仅更新状态，不发布控制
            self.update_state()
            # self.compose_control(0,0,self.current_yaw,0,1)
        
            
        except tf2_ros.TransformException as e:
            rospy.logwarn(f"Transform exception: {e}")
            return



  
        

    def check_data_expiry(self):
        """清除过期数据"""
        current_time = rospy.Time.now()
        for marker_type in ['left', 'right', 'center', 'center_left', 'center_right']:
            # rospy.loginfo(f"{marker_type} current_time: {current_time} marker_time: {self.marker_time[marker_type]}")
                
            if self.marker_time[marker_type] and \
               (current_time - self.marker_time[marker_type]).to_sec() > self.data_expiry:
                
                self.markers[marker_type] = None
                self.marker_time[marker_type] = None
                # rospy.loginfo(f"清除过期标记数据: {marker_type}")

#------------------------------------CALCULATION---------------------------------------------------------------------------------------------------

    def test_center_side_target(self, side):
        pose = self.markers[side]
        orientiation = self.markers_orientation[side]
        pos = np.array([pose.x, pose.y, pose.z])
        rot = orientiation
        #计算转换到基坐标系下的旋转矩阵
        R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        # 计算中间位置 * sign
        sign = 1 if side == 'right' else -1
        offset = self.marker_side_spacing/2 *sign+0.03
        #计算目标点---中心点前面stop_distance的点
        self.pos_target = R @ np.array([-offset, 0, self.stop_distance]) + pos
        #计算中间点
        pos_center = R @ np.array([-offset, 0, 0]) + pos
        return {
            'position': self.pos_target,
            'yaw': self.get_marker_yaw(self.pos_target),
            'center': pos_center
        }

    def calculate_center_front_target(self, side):
        pose = self.markers[side]
        rot = self.markers_orientation[side]
        pos = np.array([pose.x, pose.y, pose.z])
        R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        sign = 1 if side == 'left' else -1
        offset = self.marker_spacing/2
        #沿着物体当前坐标系下进行变换(camera_link?)
        self.pos_target = R @ np.array([-2.5*sign, 0, 0 ]) + pos
        pos_center =  R @ np.array([0 , 0, -offset]) +pos

        return {
            'position': self.pos_target,
            'yaw': self.get_marker_yaw(self.pos_target),
            'center': pos_center
        }
    def get_side_center_angle(self, side_target):
        #利用余弦定理计算∠(原点, 标记位置, 中心点)的角度
        #       标记位置 (pos:side_target_pos)
        #          *
        #         /-\
        #        /   \
         # l1   /     \  l3
        #      /       \
        #     /         \
        #    *-----------*
        # 原点      中心点 (pos_center)
        #          l2
        l1 = np.linalg.norm(side_target['position'][:2])
        l2 = np.linalg.norm(side_target['center'][:2])

        l3 = np.linalg.norm(side_target['position'][:2]-side_target['center'][:2])
        # cos_theta = (l1^2 + l3^2 - l2^2)/(2*l1*l3)   # ^---bitwise_xor
        cos_theta = (l1**2 + l3**2 - l2**2)/(2*l1*l3)
        theta = np.arccos(cos_theta)
        #航向角 弧度
        yaw = np.pi - theta
        if self.markers['leftside'] is None:
            yaw = -yaw
        return yaw

    def calculate_center_side_target(self, side):
        """计算中间标记前的目标点（基于单侧标记）"""
        # pose_stamped=self.markers_orientation[side]
        # rospy.loginfo(f"pose_stamped: {self.markers_orientation[side]}")
        # if self.markers_orientation[side] is None:
        #     return None
        # pose = self.markers[side]
        # rospy.loginfo(f"pose: {pose}")
        if self.markers_orientation[side] is None or self.markers[side] is None:
            return None

        # 从 PoseStamped 中提取位置和方向
        pose = self.markers[side]
        orientation = self.markers_orientation[side]
        # rospy.loginfo(f"pose: {pose}")

            
        pos=np.array([pose.x,pose.y,pose.z])
        rot=orientation
        R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        sign = 1 if side == 'right' else -1
        # 计算中间位置 * sign
        offset = self.marker_side_spacing/2 *sign+0.03
        #计算目标点---中心点前面stop_distance的点
        self.pos_target = R@np.array([-offset, 0,self.stop_distance]) + pos
        pos_center = R@np.array([-offset,0, 0]) + pos
        # rospy.loginfo(f"pos: {pos}")
        # rospy.loginfo(f"self.pos_target : {self.pos_target }")

        return {
            'position': self.pos_target,
            'yaw': self.get_marker_yaw(self.pos_target),
            'center': pos_center,
            # 'yaw': np.arctan2(marker['position'][1], marker['position'][0]) + np.pi/2
        }
    
    def calculate_back_side_target(self, side):
        """计算中间标记前的目标点（基于单侧标记）"""
        marker = self.markers[side]
        # pos = marker['position']
        # rot = marker['orientation']
        
        pose_stamped=self.get_rot(self.markers[side])
        if pose_stamped is None:
            return None
        pose = pose_stamped.pose
        # if side == 'back_left':
        #     self.pose2_pub.publish(pose_stamped)
        # else:
        #     self.pose3_pub.publish(pose_stamped)
            
        pos=np.array([pose.position.x,pose.position.y,pose.position.z])
        rot=pose.orientation
        R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        sign = 1 if side == 'back_right' else -1
        # 计算中间位置 * sign
        offset = self.marker_side_spacing/2 *sign+0.03
        self.pos_target = R@np.array([-offset, 0,self.stop_distance+0.9]) + pos
        pos_center = R@np.array([-offset,0, 0.9]) + pos
        # rospy.loginfo(f"pos: {pos}")
        # rospy.loginfo(f"self.pos_target : {self.pos_target }")

        return {
            'position': self.pos_target,
            'yaw': self.get_marker_yaw(self.pos_target),
            'center': pos_center,
            # 'yaw': np.arctan2(marker['position'][1], marker['position'][0]) + np.pi/2
        }

    
    def rotation_matrix_to_quaternion(self,R):
        # 旋转矩阵转欧拉角（弧度）
        # 计算四元数分量的平方
        qw_sq = 0.25 * (1 + R[0,0] + R[1,1] + R[2,2])
        qx_sq = 0.25 * (1 + R[0,0] - R[1,1] - R[2,2])
        qy_sq = 0.25 * (1 - R[0,0] + R[1,1] - R[2,2])
        qz_sq = 0.25 * (1 - R[0,0] - R[1,1] + R[2,2])
        
        # 选择最大分量避免除以零
        max_idx = np.argmax([qw_sq, qx_sq, qy_sq, qz_sq])
        if max_idx == 0:
            qw = np.sqrt(qw_sq)
            qx = (R[2,1] - R[1,2]) / (4 * qw)
            qy = (R[0,2] - R[2,0]) / (4 * qw)
            qz = (R[1,0] - R[0,1]) / (4 * qw)
        elif max_idx == 1:
            qx = np.sqrt(qx_sq)
            qw = (R[2,1] - R[1,2]) / (4 * qx)
            qy = (R[1,0] + R[0,1]) / (4 * qx)
            qz = (R[0,2] + R[2,0]) / (4 * qx)
        elif max_idx == 2:
            qy = np.sqrt(qy_sq)
            qw = (R[0,2] - R[2,0]) / (4 * qy)
            qx = (R[1,0] + R[0,1]) / (4 * qy)
            qz = (R[2,1] + R[1,2]) / (4 * qy)
        else:
            qz = np.sqrt(qz_sq)
            qw = (R[1,0] - R[0,1]) / (4 * qz)
            qx = (R[0,2] + R[2,0]) / (4 * qz)
            qy = (R[2,1] + R[1,2]) / (4 * qz)
        return np.array([qx, qy, qz, qw])  # 返回 [x, y, z, w]
   
    def axes_to_quaternion(self,x_axis, y_axis, z_axis):
        """
        将坐标轴方向向量转换为四元数
        :param x_axis: X轴方向向量 (np.array, shape=(3,))
        :param y_axis: Y轴方向向量 (np.array, shape=(3,))
        :param z_axis: Z轴方向向量 (np.array, shape=(3,))
        :return: geometry_msgs/Quaternion 四元数
        """
        # 1. 检查输入向量是否正交且单位化
        tolerance = 1e-6
        if not (np.abs(np.dot(x_axis, y_axis)) < tolerance and
                np.abs(np.dot(y_axis, z_axis)) < tolerance and
                np.abs(np.dot(z_axis, x_axis)) < tolerance):
            raise ValueError("输入向量不正交！")
        
        if not (np.isclose(np.linalg.norm(x_axis), 1.0, atol=tolerance) and
                np.isclose(np.linalg.norm(y_axis), 1.0, atol=tolerance) and
                np.isclose(np.linalg.norm(z_axis), 1.0, atol=tolerance)):
            raise ValueError("输入向量未单位化！")
        
        # 2. 构建旋转矩阵 (3x3)
        rotation_matrix = np.eye(4)  # 扩展为齐次坐标矩阵
        rotation_matrix[:3, 0] = x_axis  # 第一列为X轴
        rotation_matrix[:3, 1] = y_axis  # 第二列为Y轴
        rotation_matrix[:3, 2] = z_axis  # 第三列为Z轴
        
        # 3. 旋转矩阵转四元数
        q = self.rotation_matrix_to_quaternion(rotation_matrix)
        
        # 4. 转换为ROS Quaternion消息
        quaternion_msg = Quaternion()
        quaternion_msg.x = q[0]
        quaternion_msg.y = q[1]
        quaternion_msg.z = q[2]
        quaternion_msg.w = q[3]
        return quaternion_msg

    
   
    def calculate_center_target(self):
        """计算中间标记前的目标点"""

        # pose_stamped=self.get_rot(self.markers['center'])
        # if pose_stamped is None:    
        #     return None 
        # pose = pose_stamped.pose
        # self.pose1_pub.publish(pose_stamped)
        # pos=np.array([pose.position.x,pose.position.y,pose.position.z])
        # rot=pose.orientation
        if self.markers['center'] is None:
            return None
        pose = self.markers['center']
        pos=np.array([pose.x,pose.y,pose.z])
        rot=self.markers_orientation['center']


        R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        #计算目标点---中心点前面stop_distance的点
        self.pos_target = R@[-self.center_side_offset[1],0 , self.stop_distance] + pos

        pos =  R@[-0.05,0 ,0] + pos

    
        return {
            'position': self.pos_target,
            'yaw': self.get_marker_yaw(self.pos_target),
            'center': pos,
        }

    def calculate_midpoint(self):
        """计算左右标记中点"""
        left = self.markers['left']['position']
        right = self.markers['right']['position']
        mid_pos = (left + right) / 2
        return {
            'position': np.array([mid_pos[0] - self.target_distance, mid_pos[1], 0]),
            'yaw': (self.get_marker_yaw(self.markers['left']) + 
                   self.get_marker_yaw(self.markers['right'])) / 2
        }

    def estimate_center(self, side):
        """估计中间位置（基于单侧标记）"""
        marker = self.markers[side]
        # pos = self.markers[side]['position']
        # rot = self.markers[side]['orientation']
        pose=self.get_rot(self.markers[side])
        pos=np.array([pose.position.x,pose.position.y,pose.position.z])
        rot=pose.orientation

        R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        sign = 1 if side == 'right' else -1
        # 计算中间位置 * sign
        offset = -self.marker_spacing/2*sign
        self.pos_target = R@[-sign*self.stop_distance,0, -offset] + pos
        pos_center = R@[0,0, offset] + pos
        return {
            'position': self.pos_target,
            'yaw': self.get_marker_yaw(self.pos_target),
            'center': pos_center
            # 'yaw': np.arctan2(marker['position'][1], marker['position'][0]) + np.pi/2
        }

    def calculate_docking_target(self):
        """计算中间标记前的目标点"""

        marker_distance = math.sqrt(self.markers['center']['position'][0]**2 + self.markers['center']['position'][1]**2)
        if marker_distance > self.target_distance:
            pos = self.markers['center']['position']
            # 保持目标距离（沿X轴）
            rot = self.markers['center']['orientation']
            R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
            # self.pos_target = R@[0, 0, self.stop_distance] + pos
            self.pos_target = pos

            if self.get_marker_yaw(self.markers['center']) is None:
                rospy.logwarn("无法获取航向角")
                return None
            return {
                'position': self.pos_target,
                # 'position': np.array([pos[0] - self.target_distance, pos[1], 0]),
                'yaw': self.get_marker_yaw(self.markers['center'])
            }

    def get_marker_yaw(self, pos_target):
        """从标记位姿获取航向角（已通过TF旋转补偿）"""
        t = pos_target
        yaw = math.atan2(t[1], t[0])
        
        # _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw  
    
    def yaw_to_target_yaw_angle(self, yaw, current_yaw):
        """将航向角转换为控制角度"""
        # rospy.loginfo(f"current_yaw: {self.current_yaw}")
        # imu ccw and cw !!!!!! 记得根据实际情况修改 九洲需要加-
        angle= self.angle_dir*(math.degrees(yaw)*100) + math.degrees(current_yaw)*100
        #计算gps距离
        # rospy.loginfo(f"angle: {angle}")
        if angle > 36000:
            angle -= 36000
        if angle < 0:
            angle += 36000
        # rospy.loginfo(f"angle: {angle}")
        return np.uint16(angle)

    def apply_yaw_filter(self, raw_yaw):
        """应用组合滤波器到原始航向角"""
        if not self.filter_enabled:
            return raw_yaw
        
        current_time = rospy.Time.now()
        if self.last_filter_time is None:
            # 首次滤波初始化
            self.filtered_yaw = raw_yaw
            self.last_filter_time = current_time
            return raw_yaw
        
        # 计算时间间隔
        dt = (current_time - self.last_filter_time).to_sec()
        self.last_filter_time = current_time
        
        #计算gps距离
        # 低通滤波公式
        alpha = dt / (self.filter_time_constant + dt)
        lowpass_yaw = alpha * raw_yaw + (1 - alpha) * self.filtered_yaw
        
        # EMA二次平滑
        self.filtered_yaw = self.ema_alpha * lowpass_yaw + (1 - self.ema_alpha) * self.filtered_yaw
        
        rospy.logdebug(f"滤波前: {math.degrees(raw_yaw):.2f}°, 滤波后: {math.degrees(self.filtered_yaw):.2f}°")
        return self.filtered_yaw

    def gps_calculation(self, lat1, lon1, lat2, lon2):
        utm1 = utm.fromLatLong(lat1, lon1)
        utm2 = utm.fromLatLong(lat2, lon2)
        
        easting_diff = utm2.easting - utm1.easting
        northing_diff = utm2.northing - utm1.northing
        rospy.loginfo(f"easting_diff: {easting_diff} northing_diff: {northing_diff} gps distance:{self.distance2drone} yaw: {self.yaw2drone}")
        # 航向角（北向为0，北偏东为正0-360）
        # 计算无人机相对于基坐标系的坐标
        self.distance2drone = math.sqrt(easting_diff**2 + northing_diff**2)
        self.yaw2drone = math.atan2(easting_diff, northing_diff)
     
    def search(self):
        #找不到，基于与无人机朝向，左右旋转np.pi/20弧度。
        self.current_target=None
        self.state="SEARCH"
        if self.distance2drone > 1.0 or self.distance2drone <=0.1:
            control = controlData()
            control.distance = 0 
            # control.target_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            # control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            if self.current_yaw>self.yaw2drone:
                up_yaw=-np.pi/20
            else:
                up_yaw=np.pi/20
            control.target_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, up_yaw)
            control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            control.roller_speed = 0
            control.robot_state = 1
            self.control_pub.publish(control)
            time.sleep(0.05)
            control.robot_state = 2
            self.control_pub.publish(control)
            time.sleep(0.5)

            self.control_seq += 1
        else:
            control = controlData()
            control.distance = -100 
            # control.target_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            # control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            control.target_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            control.roller_speed = 0
            control.robot_state = 2
            self.control_pub.publish(control)
            self.control_seq += 1
            time.sleep(0.5)

        return control

    def get_pre_robot_pose(self,):
        
        v1=self.current_target['position'][:2]
        v2=self.current_target['center'][:2]
        axis=v1-v2
        axis/=np.linalg.norm(axis)
        prepoint=v1+axis*0.3
        costh=np.dot(prepoint,axis)/np.linalg.norm(prepoint)
        theta2=math.acos(costh)
        #axis=-axis
        if (prepoint[0]*axis[1]-prepoint[1]*axis[0])<0:
            theta2=-theta2
        distance=-np.linalg.norm(prepoint)
        # theta1=math.atan(abs(prepoint[1]/prepoint[0]))

        # if prepoint[1]>0:
        #     theta1=-theta1
        theta1 = math.atan2(-prepoint[1], -prepoint[0])
        return distance,theta1,theta2

    def get_step1_robot_pose(self,current_target):
        
        v1=current_target['position'][:2]
        v2=current_target['center'][:2]
        axis=v1-v2
        axis/=np.linalg.norm(axis)
        prepoint=v1+axis*0.3
        costh=np.dot(prepoint,axis)/np.linalg.norm(prepoint)
        theta2=math.acos(costh)
        #axis=-axis
        if (prepoint[0]*axis[1]-prepoint[1]*axis[0])<0:
            theta2=-theta2
        if prepoint[0] < 0 :
            distance=-np.linalg.norm(prepoint)
            theta1 = math.atan2(-prepoint[1], -prepoint[0])
        else:
            distance=np.linalg.norm(prepoint)
            theta1 = math.atan2(prepoint[1], prepoint[0])
        # theta1=math.atan(abs(prepoint[1]/prepoint[0]))

        # if prepoint[1]>0:
        #     theta1=-theta1
        return distance,theta1,theta2

    def direct_back(self):

        distance=-0.3
        # theta1=math.atan(abs(prepoint[1]/prepoint[0]))

        # if prepoint[1]>0:
        #     theta1=-theta1
        theta1 = theta2=   0
        return distance,theta1,theta2

    def get_step00_robot_pose(self,current_target):
        
        v1=current_target['position'][:2]
        v2=current_target['center'][:2]
        axis=v1-v2
        axis/=np.linalg.norm(axis)
        axis=-axis
        prepoint=v1-np.array([-0.3,0])

        costh=np.dot(prepoint,axis)/np.linalg.norm(prepoint)
        theta2=math.acos(costh)
        if (prepoint[0]*axis[1]-prepoint[1]*axis[0])<0:
            theta2=-theta2
        distance=np.linalg.norm(prepoint)
        # theta1=math.atan(abs(prepoint[1]/prepoint[0]))
        # if prepoint[1]<0:
        #     theta1=-theta1
        theta1 = math.atan2(prepoint[1], prepoint[0])
        
        
        return distance,theta1,theta2    

    def get_step2_robot_pose(self,current_target):
        
        v1=current_target['position'][:2]
        v2=current_target['center'][:2]
        axis=v1-v2
        axis/=np.linalg.norm(axis)
        axis=-axis
        prepoint=v1

        costh=np.dot(prepoint,axis)/np.linalg.norm(prepoint)
        theta2=math.acos(costh)
        if (prepoint[0]*axis[1]-prepoint[1]*axis[0])<0:
            theta2=-theta2
        distance=np.linalg.norm(prepoint)
        # theta1=math.atan(abs(prepoint[1]/prepoint[0]))
        # if prepoint[1]<0:
        #     theta1=-theta1
        theta1 = math.atan2(prepoint[1], prepoint[0])
        
        
        return distance,theta1,theta2    

    def get_five_avg(self):
        target_list=[]
        y_list=[]
        for i in range(11):
            loc=copy.deepcopy(self.current_target)
            target_list.append(loc)
            y_list.append(loc['center'][1])
            time.sleep(0.1)
        
        indexed = list(enumerate(y_list))
        sorted_with_indices = sorted(indexed, key=lambda x: x[1])
        
        # 计算中间 5 个元素的起始位置
        n = len(sorted_with_indices)
        start = (n - 5) // 2
        middle_five = sorted_with_indices[start : start+5]
        # 提取原始索引
        midde_five=[index for index, value in middle_five]
        outtarget={'position': np.array([0,0,0.0]),
                   'center': np.array([0,0,0.0])}
        for i in range(5):
            item=target_list[midde_five[i]]
            outtarget['center']+=item['center']
            xaxis=item['center']-item['position']
            xaxis[2]=0
            xaxis/=np.linalg.norm(xaxis)
            outtarget['position']+=xaxis
        outtarget['center']/=5
        outtarget['position']/=5
        xaxis/=np.linalg.norm(outtarget['position'])
        outtarget['position']=outtarget['center']-xaxis*self.stop_distance


        return outtarget
#------------------------------------FUNCTION---------------------------------------------------------------------------------------------------

    def compose_control(self,distance,roller_speed,yaw,target_yaw_diff,robot_state):
        control = controlData()
        control.distance = distance
        control.roller_speed = roller_speed
        control.yaw = self.yaw_to_target_yaw_angle(yaw,0)
        control.target_yaw = self.yaw_to_target_yaw_angle(target_yaw_diff,yaw)
        control.robot_state = robot_state
        control.header.stamp = rospy.Time.now()
        control.header.seq = self.control_seq
        rospy.loginfo(f"compose_control robot_state: {(control.robot_state)} ")
        # control.header.stampd
        return control
    
    

#------------------------------------CONTROL---------------------------------------------------------------------------------------------------

    def control_loop_test(self, event):
        """主控制循环""" 
        control = controlData()
        aruco_test=False  #默认False
        if self.stop_flag == False:
            self.control_seq += 1
            if self.rc_control == 1:   

                if self.docking_flag ==False:

                    self.update_state()
                    control = controlData()
                    control.distance = 0
                    self.gps_calculation(self.latitude, self.longitude, self.latitude_drone, self.longitude_drone)
                    # 返回distance2drone、yaw2drone
                    
                    if self.distance2drone > 1.0 and self.current_target is None:
                        # 通过GPS先行走到目标点前1m
                        control.distance = int(self.distance2drone*1000)
                        rospy.loginfo(f"distance: {control.distance}")
                        control.target_yaw = self.yaw_to_target_yaw_angle(self.yaw2drone, 0)
                        control.header.stamp = rospy.Time.now()
                        control.header.seq = self.control_seq
                        control.robot_state = 1
                        self.control_pub.publish(control)
                        time.sleep(0.05)
                        control.robot_state = 2
                        self.control_pub.publish(control)
                        self.control_seq += 1
                    else:
                            if self.search_count>0:
                                self.search_count+=1
                            else:
                                self.search_count=0
                                self.search()
                                # rospy.loginfo(f"---------- Search ----------")

                    #不断执行搜索---先判断前方标记、再判断侧边标记，执行不同动作
                    if (self.markers['center'] or self.markers['left'] or self.markers['right']) and self.current_target is not None:
                        
                        # self.state = "APPROACHING"
                        self.state = "SEARCH"
                        self.search_count = 0
                        rospy.loginfo('-------------front marker test-------------')
                        # self.lock_current=True #不允许currentpose改为None，可以进行更新
                        # rospy.loginfo(f"self.lock_current: {self.lock_current}")
                        current_pos = np.array([0, 0])  # 基坐标系原点
                        # 计算当前状态,行走到目标点前1m
                        target_vec = self.current_target['position'][:2] - current_pos
                        rospy.loginfo(f'--------------target_vec -------------- {target_vec}')
                        aruco_test = True
                        self.refine_align = False  

                    elif (self.markers['leftside'] or self.markers['rightside']) and \
                        not (self.markers['center'] or self.markers['left'] or self.markers['right']):
                        # if (self.markers['center'] or self.markers['left'] or self.markers['right']):
                        #     pass
                        self.state = "SEARCH"
                        rospy.loginfo('-------------side marker test-------------')
                        self.search_count=0
                        # self.lock_current=True #不允许currentpose改为None，可以进行更新
                        rospy.loginfo(f"self.lock_current: {self.lock_current}")
                        current_pos = np.array([0, 0])  # 基坐标系原点
                        # 计算当前状态,行走到目标点前1m
                        target_vec = self.side_target['position'][:2] - current_pos

                        #计算目标点运动控制
                        rospy.loginfo(f'--------------target_vec -------------- {target_vec}')
                        #发布控制由侧标记到目标点
                        if target_vec[0] > 0:
                            control.distance = round(np.linalg.norm(target_vec))
                            rospy.loginfo(f'distance:{control.distance}')
                            yaw=np.arctan2(target_vec[1],target_vec[0])
                            control.target_yaw = self.yaw_to_target_yaw_angle(yaw,self.current_yaw)
                            rospy.loginfo(f'目标点朝向角:{control.target_yaw}')
                            control.header.stamp = rospy.Time.now()
                            control.robot_state = 1
                            self.control_pub.publish(control)
                            rospy.sleep(0.05)
                            control.robot_state = 2
                            self.control_pub.publish(control)
                            rospy.sleep(0.1)
                            while self.complete_state != 2:
                                if self.rc_control == 2:
                                    break
                                continue
                            
                            #发布对齐中心点朝向角运动控制
                            control.distance = 0
                            target_yaw = self.get_side_center_angle(self.side_target)
                            control.target_yaw = self.yaw_to_target_yaw_angle(target_yaw,self.current_yaw)
                            rospy.loginfo(f'对齐中心点target_yaw:{control.target_yaw}')
                            control.header.stamp = rospy.Time.now()
                            control.robot_state = 1
                            self.control_pub.publish(control)
                            rospy.sleep(0.05)
                            control.robot_state = 2
                            self.control_pub.publish(control)
                            rospy.sleep(0.1)
                            while self.complete_state != 2:
                                if self.rc_control == 2:
                                    break
                                continue
                            self.lock_current = False
                            self.control_seq += 1 

                            aruco_test = True
                            self.refine_align = False
                    


                        
                                        
                    if aruco_test == True and self.refine_align==False :
                        """粗定位环节"""
                        # 考虑
                        # if self.markers['left'] or self.markers['right'] or self.markers['center']::
                        #忽略 远距离，测试标记粗定位
                        if np.linalg.norm(target_vec) >0.6:
                            # self.align_num=False
                            pass
                        if np.linalg.norm(target_vec)> self.stop_distance_threshold and \
                            self.align_num==False and self.current_target is not None:
                            rospy.loginfo(f"未到达目标位置: {self.current_target['position']},{self.get_marker_yaw(self.current_target['position'])}")
                            rospy.loginfo(f"complete_state: {self.complete_state}")
                            if target_vec[0]>0:
                                self.target_distance = np.linalg.norm(target_vec) 
                                self.target_distance=np.clip(self.target_distance,0,0.2)
                                # rospy.loginfo(f"target_distance: {self.target_distance}")
                                self.target_yaw = math.atan2(target_vec[1], target_vec[0])
                                self.target_yaw =np.clip(self.target_yaw,-0.2,0.2)
                                # rospy.loginfo(f"target_yaw: {self.target_yaw}")
                                if  np.linalg.norm(target_vec)<0.1:
                                    self.target_yaw=0
                            else:
                                self.target_yaw = 0
                                control.distance = -100
                                control.target_yaw = self.yaw_to_target_yaw_angle(0,self.current_yaw)
                                control.header.stamp = rospy.Time.now()
                                control.robot_state = 1
                                self.control_pub.publish(control)
                                time.sleep(0.05)
                                control.robot_state = 2 
                                self.control_pub.publish(control)
                                self.control_seq += 1
                                time.sleep(0.5)
                                self.lock_current=False
                                return 
                                
                            control.distance = int(self.target_distance*1000)
                            control.target_yaw = self.yaw_to_target_yaw_angle(self.target_yaw,self.current_yaw)
                            control.robot_state = 2

                            # 发布控制指令
                            control.header.stamp = rospy.Time.now()
                            control.header.seq = self.control_seq
                            # self.state_prev = self.state
                            rospy.loginfo(f'state: {control.robot_state}')
                            if self.complete_state==2:
                                control.robot_state = 1
                                control.header.stamp = rospy.Time.now()
                                self.control_pub.publish(control)
                                time.sleep(0.05)
                                control.robot_state = 2 
                                control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)
                            self.control_seq += 1
                        else:
                            # 对齐alig_num flag置1,并暂停机器人
                            self.align_num=True
                            control.distance = -100
                            control.target_yaw = self.yaw_to_target_yaw_angle(0,self.current_yaw)
                            control.header.stamp = rospy.Time.now()
                            control.robot_state = 1
                            self.control_pub.publish(control)
                            time.sleep(0.05)
                            control.robot_state = 2
                            self.control_pub.publish(control)
                            self.control_seq += 1
                            time.sleep(0.5)
                            self.lock_current=False
                            return
                        if self.align_num==True and self.refine_align == False:
                            #执行对齐操作
                            rospy.loginfo(f"到达目标位置！对齐操作{self.current_target['position']},{self.get_marker_yaw(self.current_target['position'])}")
                            if abs(self.get_marker_yaw('center'))<0.015:
                                rospy.loginfo(f"粗对正完成！{target_vec[0]} {target_vec[1]}")

                                self.refine_align=True

                                control.robot_state = 1
                                control.header.stamp = rospy.Time.now()
                                self.control_pub.publish(control)
                                time.sleep(0.05)
                                control.header.stamp = rospy.Time.now()
                                self.control_pub.publish(control)
                                time.sleep(0.5)
                                control.robot_state = 2
                                self.control_seq += 1
                                self.control_pub.publish(control)
                            else:
                                control.distance = 0
                                #进行纠正对齐操作
                                c_yaw = self.get_marker_yaw('center')
                                if c_yaw>0.1:
                                    c_yaw=0.1
                                elif c_yaw<-0.1:
                                    c_yaw=-0.1
                                control.target_yaw = self.yaw_to_target_yaw_angle(c_yaw,self.current_yaw)

                                control.robot_state = 1
                                control.header.stamp = rospy.Time.now()
                                self.control_pub.publish(control)
                                time.sleep(0.05)
                                control.header.stamp = rospy.Time.now()
                                self.control_pub.publish(control)
                                time.sleep(0.5)
                                control.robot_state = 2
                                self.control_seq += 1

                                self.refine_align = False
                    if self.refine_align==True and self.align_num==False:
                        #精确对正环节
                        control.distance = 0
                        control.target_yaw = 0                            
                        control.robot_state = 1
                        control.header.stamp = rospy.Time.now()
                        self.control_pub.publish(control)
                        time.sleep(0.5)
                        current_pose_state=self.get_five_avg()#取5次平均值进行计算
                        target_vec = current_pose_state['position'][:2]
                        rospy.loginfo(f'target_vec_refine: {target_vec}')
                        # if np.linalg.norm(target_vec) >1.0:

                        #     # self.refine_align=False 

                        #     self.lock_current=False
                        #     return         
                        if np.linalg.norm(target_vec)<self.stop_distance_threshold and \
                            abs(np.linalg.norm(target_vec[1]))>self.stop_refine_pose_dlt_y and \
                            np.linalg.norm(target_vec[0])>0 :
                            
                            #无需纠正
                            rospy.logwarn(f"-------------精对正完成！-------------")
                            control.robot_state = 4
                            
                            control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)
                            self.control_seq += 1
                            time.sleep(0.5)
                            self.docking_flag=True
                            self.in_dock_flag=False
                            self.lock_current=False
                        #需要继续纠正, 步骤1-回退
                        dis,yaw1,yaw2=self.get_step1_robot_pose(current_pose_state)
                        rospy.loginfo(f"dis: {dis} yaw1: {yaw1} yaw2: {yaw2}")
                        control.distance = int(dis*1000)
                        control.target_yaw = self.yaw_to_target_yaw_angle(yaw1,self.current_yaw)
                        # 发布控制指令  
                        control.robot_state = 2
                        control.header.stamp = rospy.Time.now()
                        rospy.loginfo(f'先后退: {control.robot_state}')
                        self.control_pub.publish(control)
                        time.sleep(0.5)
                        rospy.loginfo('等待回退完成...')
                        self.lock_current=False
                        while control.robot_state != 2:
                            pass
                        rospy.loginfo(f'回退完成！')
                        control.distance = 0
                        control.target_yaw = 0
                        control.robot_state = 1
                        control.header.stamp = rospy.Time.now()
                        self.control_pub.publish(control)
                        time.sleep(0.5)
                        # 步骤1-前进

                        control.target_yaw = self.yaw_to_target_yaw_angle(yaw2,self.current_yaw)
                        # 发布控制指令  
                        control.robot_state = 2
                        control.header.stamp = rospy.Time.now()
                        rospy.loginfo(f'等待回正...')
                        self.control_pub.publish(control)
                        time.sleep(0.5)
                        rospy.loginfo('回正中！')
                        self.lock_current=False
                        while control.robot_state != 2:
                            pass
                        rospy.loginfo(f'已回正, 步骤 1 结束')
                        control.distance = 0
                        control.target_yaw = 0
                        control.robot_state = 1
                        control.header.stamp = rospy.Time.now()
                        self.control_pub.publish(control)
                        time.sleep(0.5)

                        # 步骤2-前进
                        current_pose_state = self.get_five_avg()#取5次平均值进行计算
                        dis,yaw1,yaw2=self.get_step2_robot_pose(current_pose_state)
                        control.distance = int(dis*1000)
                        control.target_yaw = self.yaw_to_target_yaw_angle(yaw1,self.current_yaw)
                        # 发布控制指令
                        control.robot_state = 2
                        control.header.stamp = rospy.Time.now()
                        rospy.loginfo(f'前进: {control.robot_state}')
                        self.control_pub.publish(control)
                        time.sleep(0.5)
                        rospy.loginfo('前进中！')
                        self.lock_current=False
                        while control.robot_state != 2:
                            pass
                        rospy.loginfo(f'前进完成！')
                        control.distance = 0
                        control.target_yaw = 0
                        control.robot_state = 1
                        control.header.stamp = rospy.Time.now()
                        self.control_pub.publish(control)
                        time.sleep(0.5)
                        # 步骤2-前进后-回正
                        control.target_yaw = self.yaw_to_target_yaw_angle(yaw2,self.current_yaw)
                        # 发布控制指令  
                        control.robot_state = 2
                        control.header.stamp = rospy.Time.now()
                        rospy.loginfo(f'等待回正...')
                        self.control_pub.publish(control)
                        time.sleep(0.5)
                        rospy.loginfo('回正中！')
                        self.lock_current=False
                        while control.robot_state != 2:
                            pass
                        rospy.loginfo(f'已回正, 步骤 2 结束')
                        control.distance = 0
                        control.target_yaw = 0
                        control.robot_state = 1
                        control.header.stamp = rospy.Time.now()
                        self.control_pub.publish(control)
                        time.sleep(0.5)

                        while False:
                            self.lock_current = False
                            current_pose_state = self.get_five_avg()#取5次平均值进行计算
                            target_vec = current_pose_state['position'][:2]
                            if abs(self.get_marker_yaw(current_pose_state['center'])) < 0.015 :
                                rospy.loginfo(f"精对齐修正完成! target_vec: {target_vec[0]} {target_vec[1]}")
                                self.refine_align = True #测试注释后的情况
                                control.distance = 0
                                control.target_yaw = 0
                                control.robot_state = 1
                                control.header.stamp = rospy.Time.now()
                                self.control_pub.publish(control)
                                time.sleep(0.5)
                                break
                            else:
                                c_yaw=self.get_marker_yaw(current_pose_state['center'])
                                if c_yaw>0.1:
                                    c_yaw=0.1
                                elif c_yaw<-0.1:
                                    c_yaw=-0.1
                                control.target_yaw = self.yaw_to_target_yaw_angle(c_yaw,self.current_yaw)
                                control.robot_state = 1
                                control.header.stamp = rospy.Time.now()
                                self.control_pub.publish(control)
                                time.sleep(0.5)
                                control.robot_state = 2
                                control.header.stamp = rospy.Time.now()
                                self.control_pub.publish(control)
                                time.sleep(0.5)
                                self.control_seq += 1
                        self.lock_current=False
                if self.in_dock_flag == False:
                    if self.count==0:
                        control = self.compose_control(0,0,self.current_yaw,0,1)
                        self.control_pub.publish(control)
                        time.sleep(0.1)
                        self.count = 1
                    control = self.compose_control(0,0,self.current_yaw,0,4)
                    self.control_pub.publish(control)
                    time.sleep(0.1)
                    while self.complete_state ==0:#等待对接完成
                        if self.rc_control == 0: #被遥控器中断
                            rospy.logwarn("rc_control is 0")
                            return
                        pass
                    if self.complete_state ==4:#对接成功返回4
                        self.in_dock_flag = True
                        self.count = 0
                        while self.rc_control != 2: #遥控器确认信号 2
                            if self.rc_control == 0:
                                rospy.logwarn("rc_control is 0")
                                return
                            #保持静止
                            control = self.compose_control(0,0,self.current_yaw,0,1)
                            self.control_pub.publish(control)
                            time.sleep(0.1)
                    else:
                        self.error = 1 #对接失败
                        control = self.compose_control(0,0,self.current_yaw,0,1)
                        self.control_pub.publish(control)
            elif self.rc_control == 2:
                if self.in_dock_flag == False:  # state 3
                    control.distance = 0
                    control.target_yaw = 0
                    control.robot_state = 1
                    control.roller_speed = 0
                    control.header.stamp = rospy.Time.now()
                    control.yaw =self.yaw_to_target_yaw_angle(self.current_yaw,0)

                    self.control_pub.publish(control)
                    self.latitude_drone = self.latitude
                    self.longitude_drone = self.longitude

                    time.sleep(0.1)
                    control.robot_state = 3  #退出对接，出仓
                    control.header.stamp = rospy.Time.now()
                    self.control_pub.publish(control)

                    while self.complete_state != 3:
                        if self.rc_control == 0:
                            rospy.logwarn("rc_control is 0")
                            return
                        pass
                    if self.complete_state == 3: #完成出仓
                        self.in_dock_flag = True
                        self.out_dock_flag = True
                        self.docking_flag = False
                        if self.latitude !=0 and self.longitude !=0:
                            self.latitude_drone = self.latitude
                            self.longitude_drone = self.longitude
                        self.count = 0
                    else:
                        self.error = 1

                if self.corner_finding_flag == False: # state 7
                    control.distance = 0
                    control.robot_state = 1
                    control.yaw =self.yaw_to_target_yaw_angle(self.current_yaw,0)
                    control.roller_speed = 0
                    control.header.stamp = rospy.Time.now()
                    self.control_pub.publish(control)
                    time.sleep(0.1)

                    control.robot_state = 7 #执行寻找角落
                    control.header.stamp = rospy.Time.now()
                    self.control_pub.publish(control)
                    while self.complete_state != 7:
                        if self.rc_control == 0:
                            rospy.logwarn("rc_control is 0")
                            return
                        pass
                    if self.complete_state == 7: #完成寻找角落
                        self.corner_finding_flag = True
                        self.count = 0
                    else:
                        self.error = 1
                    return

                if self.auto_cleaning_flag == False:  # state 8
                    control.distance = 0
                    control.robot_state = 1
                    control.yaw =self.yaw_to_target_yaw_angle(self.current_yaw,0)
                    control.roller_speed = 2600
                    control.header.stamp = rospy.Time.now()
                    self.control_pub.publish(control)
                    time.sleep(0.1)

                    control.robot_state = 8 #执行自动清扫
                    if self.complete_state ==8:
                        self.auto_cleaning_flag = True
                        self.count = 0 

                        while self.rc_control != 1: 
                            if self.rc_control == 0:
                                rospy.logwarn("rc_control is 0")
                                return
                            control.header.stamp = rospy.Time.now()
                            control.yaw =self.yaw_to_target_yaw_angle(self.current_yaw,0)
                            control.roller_speed = 0
                            control.robot_state = 1

                            self.control_pub.publish(control)
                            time.sleep(0.1)
                            pass
                    else:
                        self.error = 1
                        return                        
            else:
                control = self.compose_control(0,0,self.current_yaw,0,1)
                self.control_pub.publish(control)
                return
        else:
            control = self.compose_control(0,0,self.current_yaw,0,1)
            self.control_pub.publish(control)
            return   
                    

    def control_loop(self, event):
        """主控制循环""" 
        
        control = controlData()
        # rospy.loginfo(f"in_dock_flag: {self.in_dock_flag} docking_flag: {self.docking_flag} rc_control: {self.rc_control}")
        if self.stop_flag == False: #是否进入停止状态
            self.control_seq += 1
            if self.rc_control == 1:
                # self.out_dock_flag = False
                # self.corner_finding_flag = False
                # self.auto_cleaning_flag = False

                if self.docking_flag ==False: #todo 
                    self.update_state()
                    control = controlData()
                    control.distance = 0
                    #1.计算gps距离
                    self.gps_calculation(self.latitude, self.longitude, self.latitude_drone, self.longitude_drone)
                    # rospy.loginfo(f"gps_calculation: {gps_calculation}")
                    if self.distance2drone > 1 and self.current_target is None: #gps距离大于2米,通过gps数据大致导航
                        control.distance = np.uint16((self.distance2drone)*1000)
                        # rospy.loginfo(f"gps_yaw: {self.yaw_to_target_yaw_angle(self.yaw2drone, 0)}")
                        # rospy.loginfo(f"gps_distance: {self.distance2drone}")
                        control.target_yaw = self.yaw_to_target_yaw_angle(self.yaw2drone, 0)
                        control.robot_state = 2

                        # 发布控制指令
                        control.header.stamp = rospy.Time.now()
                        control.header.seq = self.control_seq
                        self.state_prev = self.state
                        rospy.loginfo(f'state: {control.robot_state}')
                        if self.complete_state==2:
                            control.robot_state = 1
                            self.control_pub.publish(control)
                            control.robot_state = 2 
                            time.sleep(0.05)
                        self.control_pub.publish(control)
                    
                        self.control_seq += 1


                    else: #gps距离小于2米,通过aruco数据导航

                        #rospy.loginfo(f'state {self.state}')
                        #2.1 执行搜索逻辑,持续20次，1s未检测到marker 进行搜索。
                        if self.markers['left'] or self.markers['right'] or self.markers['center']:
                            self.state = "APPROACHING"
                            self.search_count=0
                        else:
                            if self.search_count<20:
                                self.search_count+=1
                            else:
                                self.search_count=0
                                rospy.loginfo(f'SEARCH******************* {self.state}')
                                self.search()
                            return

                        if self.current_target:
                            self.lock_current=True #不允许currentpose改为None，可以进行更新
                            current_pos = np.array([0, 0])  # 基坐标系原点
                            # 计算当前状态,行走到目标点前1m
                            target_vec = self.current_target['position'][:2] - current_pos
                            rospy.loginfo(f'target_vec is %%%%%%%%%%% {target_vec}')
                            

                            #2.2 粗定位
                            if self.refine_align==False:
                                #2.2.1位置靠近
                                if np.linalg.norm(target_vec) >0.6:
                                    self.align_num=False
                                    # pass
                                if np.linalg.norm(target_vec) > self.stop_distance_threshold and self.align_num==False:
                                    rospy.loginfo(f"未到达目标位置: {self.current_target['position']},{self.get_marker_yaw(self.current_target['position'])}")
                                    rospy.loginfo(f"complete_state: {self.complete_state}")
                                    if target_vec[0]>0:
                                        self.target_distance = np.linalg.norm(target_vec) 
                                        self.target_distance=np.clip(self.target_distance,0,0.2)
                                        # rospy.loginfo(f"target_distance: {self.target_distance}")
                                        self.target_yaw = math.atan2(target_vec[1], target_vec[0])
                                        self.target_yaw =np.clip(self.target_yaw,-0.2,0.2)
                                        # rospy.loginfo(f"target_yaw: {self.target_yaw}")
                                        if  np.linalg.norm(target_vec)<0.1:
                                            self.target_yaw=0
                                    else:
                                        self.target_yaw = 0
                                        control.distance = -100
                                        control.target_yaw = self.yaw_to_target_yaw_angle(0,self.current_yaw)
                                        control.header.stamp = rospy.Time.now()
                                        control.robot_state = 1
                                        self.control_pub.publish(control)
                                        time.sleep(0.05)
                                        control.header.stamp = rospy.Time.now()
                                        control.robot_state = 2 
                                        self.control_pub.publish(control)
                                        self.control_seq += 1
                                        time.sleep(0.5)
                                        self.lock_current=False
                                        return 
                                        
                                    control.distance = int(self.target_distance*1000)
                                    control.target_yaw = self.yaw_to_target_yaw_angle(self.target_yaw,self.current_yaw)
                                    control.robot_state = 2

                                    # 发布控制指令
                                    control.header.stamp = rospy.Time.now()
                                    control.header.seq = self.control_seq
                                    self.state_prev = self.state
                                    rospy.loginfo(f'state: {control.robot_state}')
                                    if self.complete_state==2:
                                        control.robot_state = 1
                                        control.header.stamp = rospy.Time.now()
                                        self.control_pub.publish(control)
                                        time.sleep(0.05)
                                        control.robot_state = 2 
                                        control.header.stamp = rospy.Time.now()
                                    self.control_pub.publish(control)
                                    self.control_seq += 1
                                    

                                    
                                else:
                                    #2.2.2对齐alig_num flag置1,并暂停机器人
                                    if self.align_num==False:
                                        control.robot_state = 1
                                        control.header.stamp = rospy.Time.now()
                                        control.header.seq = self.control_seq
                                        self.control_pub.publish(control)
                                        time.sleep(0.05)
                                        control.header.stamp = rospy.Time.now()
                                        self.control_pub.publish(control)
                                        self.control_seq += 1
                                        self.align_num=True
                                        time.sleep(0.5)
                                        self.lock_current=False
                                        return
                                # 2.2.3 对齐align_num 为真,执行对齐动作
                                if self.align_num==True:
                                    
                                    rospy.loginfo(f"到达目标位置: {self.current_target['center']},{self.get_marker_yaw(self.current_target['center'])}")
                                    rospy.loginfo(f"到达目标位置__yaw: {self.current_yaw}")

                                    if abs(self.get_marker_yaw(self.current_target['center'])) < 0.015:
                                            rospy.logwarn(f"完成对正 TTTTTT:  {target_vec[0]} {target_vec[1]}")

                                            self.refine_align=True
                                            
                                            control.robot_state = 1
                                            control.header.stamp = rospy.Time.now()
                                            self.control_pub.publish(control)
                                            time.sleep(0.05)
                                            control.header.stamp = rospy.Time.now()
                                            self.control_pub.publish(control)
                                            time.sleep(0.5)
                                            control.robot_state = 2
                                            self.control_seq += 1
                                            self.control_pub.publish(control)

                                        #return
                                    else:
                                        control.distance = 0
                                        c_yaw=self.get_marker_yaw(self.current_target['center'])
                                        if c_yaw>0.1:
                                            c_yaw=0.1
                                        if c_yaw<-0.1:
                                            c_yaw=-0.1
                                        control.target_yaw = self.yaw_to_target_yaw_angle(c_yaw,self.current_yaw)
                                        control.robot_state = 2
                                        rospy.loginfo(f"real_target_yaw:{ control.target_yaw}, ￥￥￥￥￥￥curent_yaw: {self.current_yaw}")
                                        # 发布控制指令
                                        control.header.stamp = rospy.Time.now()
                                        control.header.seq = self.control_seq
                                        self.state_prev = self.state
                                        rospy.loginfo(f'state: {control.robot_state}')
                                        if self.complete_state==2:
                                            control.robot_state = 1
                                            control.header.stamp = rospy.Time.now()
                                            self.control_pub.publish(control)
                                            time.sleep(0.02)
                                            control.robot_state = 2 
                                            control.header.stamp = rospy.Time.now() 
                                        self.control_pub.publish(control)
                                    
                                        self.control_seq += 1
                                        self.refine_align=False



                            #2.3,精确对正。            
                            if self.refine_align==True:
                                        #精确对齐
                                        control.distance = 0
                                        control.target_yaw = 0                            
                                        control.robot_state = 1
                                        control.header.stamp = rospy.Time.now()
                                        self.control_pub.publish(control)
                                        time.sleep(0.5)
                                        current_pose_state=self.get_five_avg()#取5次平均值进行计算
                                        target_vec = current_pose_state['position'][:2]
                                        rospy.loginfo(f'target_vec_refine: {target_vec}')
                                        if np.linalg.norm(target_vec) >1.0:

                                            self.refine_align=False 

                                            self.lock_current=False
                                            return 

                                        if np.linalg.norm(target_vec) <self.stop_distance_threshold and abs(target_vec[1])<self.stop_refine_pose_dlt_y and np.linalg.norm(target_vec) >0: 


                                            control.robot_state = 4
                                            rospy.loginfo(f'************GOOD start final docking**************')
                                            control.header.stamp = rospy.Time.now()
                                            self.control_pub.publish(control)
                                            self.control_seq += 1 
                                            # time.sleep(1000)
                                            self.docking_flag=True
                                            self.in_dock_flag=False     
                                            self.lock_current=False                          
                                            return 
                                        

                                        #3.1 step1 
                                        d1,yaw1,yaw2=self.get_step1_robot_pose(current_pose_state)
                                        # yaw1机器人当前位置与目标点之间的夹角，用于对准目标点
                                        # yaw2目标点与中心点之间的夹角，使其对准中心点
                                        #d1,yaw1,yaw2=self.direct_back()
                                        rospy.loginfo(f'robot pose1: {d1} {yaw1} {yaw2}')
                                        control.distance = int(d1*1000)
                                        control.target_yaw = self.yaw_to_target_yaw_angle(yaw1,self.current_yaw)
                                        control.robot_state = 2
                                        control.header.stamp = rospy.Time.now()
                                        self.control_pub.publish(control)
                                        time.sleep(0.5)
                                        rospy.loginfo(f'等待回退结束 ')
                                        while self.complete_state != 2:
                                            # time.sleep(0.1)
                                            pass
                                        rospy.loginfo(f'成功回退！！ ')
                                        #执行结束
                                        control.distance = 0
                                        control.target_yaw = 0                            
                                        control.robot_state = 1
                                        control.header.stamp = rospy.Time.now()
                                        self.control_pub.publish(control)
                                        time.sleep(0.1)
                                        self.complete_state = 0
                                        control.distance = 0
                                        control.target_yaw = self.yaw_to_target_yaw_angle(yaw2,self.current_yaw)                            
                                        control.robot_state = 2
                                        control.header.stamp = rospy.Time.now()
                                        self.control_pub.publish(control)
                                        time.sleep(0.1)
                                        rospy.loginfo(f'等待回正结束 ')     
                                        while self.complete_state != 2:
                                            pass
                                        rospy.loginfo(f'step1 成功回正！ ')
                                        #执行结束


                                        #3.2 step2
                                        control.distance = 0
                                        control.target_yaw = 0                            
                                        control.robot_state = 1
                                        control.header.stamp = rospy.Time.now()
                                        self.control_pub.publish(control)
                                        time.sleep(1.0)

                                        current_pose_state=self.get_five_avg()#取5次平均值进行计算

                                        #d1,yaw1,yaw2=self.get_step2_robot_pose()
                                        d1,yaw1,yaw2=self.get_step2_robot_pose(current_pose_state)

                                        rospy.loginfo(f'robot pose22: {d1} {yaw1} {yaw2}')
                                        control.distance = int(d1*1000)
                                        control.target_yaw = self.yaw_to_target_yaw_angle(yaw1,self.current_yaw)
                                        control.robot_state = 2
                                        control.header.stamp = rospy.Time.now()
                                        self.control_pub.publish(control)
                                        time.sleep(1.0)
                                        rospy.loginfo(f'等待前进结束 ')
                                        while self.complete_state != 2:
                                            #time.sleep(0.1)
                                            pass
                                        rospy.loginfo(f'step2 成功前进！！ ')
                                        #执行结束
                                        control.distance = 0
                                        control.target_yaw = 0                            
                                        control.robot_state = 1
                                        control.header.stamp = rospy.Time.now()
                                        self.control_pub.publish(control)

                                        time.sleep(0.05)
                                        control.header.stamp = rospy.Time.now()
                                        self.control_pub.publish(control)


                                        #self.complete_state = 0
                                        control.distance = 0
                                        control.target_yaw = self.yaw_to_target_yaw_angle(yaw2,self.current_yaw)                            
                                        control.robot_state = 2
                                        control.header.stamp = rospy.Time.now()
                                        self.control_pub.publish(control)
                                        time.sleep(1.0)
                                        rospy.loginfo(f'step2 等待回正结束')
                                        # while self.complete_state != 1:
                                        #     pass
                                        rospy.loginfo(f'step2 成功回正！！')
                                        time.sleep(1.0 ) 

                                        #对齐

                                        #3.3 精对正环节下的对齐
                                        if True:
                                            self.lock_current=False
                                            
                                            while True:
                                                current_pose_state=self.get_five_avg()#取5次平均值进行计算
                                                target_vec=current_pose_state['position'][:2]
                                                rospy.loginfo(f"3.3 refine ")
                                                if abs(self.get_marker_yaw(current_pose_state['center'])) < 0.015:
                                                        rospy.logwarn(f"经修后，完成对正 2222:  {target_vec[0]} {target_vec[1]}")
                                                        self.refine_align=True
                                                        control.robot_state = 1
                                                        control.header.stamp = rospy.Time.now()
                                                        self.control_pub.publish(control)
                                                        time.sleep(0.05)
                                                        self.control_seq += 1
                                                        break

                                                    #return
                                                else:
                                                    control.distance = 0
                                                    c_yaw=self.get_marker_yaw(current_pose_state['center'])
                                                    rospy.loginfo(f'c_yaw: {c_yaw}')
                                                    if c_yaw>0.1:
                                                        c_yaw=0.1
                                                    if c_yaw<-0.1:
                                                        c_yaw=-0.1
                                                    control.target_yaw = self.yaw_to_target_yaw_angle(c_yaw,self.current_yaw)
                                                    control.robot_state = 2
                                                    # 发布控制指令
                                                    control.header.stamp = rospy.Time.now()
                                                    control.header.seq = self.control_seq
                                                    if self.complete_state==2:
                                                        control.robot_state = 1
                                                        control.header.stamp = rospy.Time.now()
                                                        self.control_pub.publish(control)
                                                        time.sleep(0.05)
                                                        control.robot_state = 2 
                                                        control.header.stamp = rospy.Time.now() 
                                                    self.control_pub.publish(control)
                                                    self.control_seq += 1

                            self.lock_current=False

                if self.in_dock_flag == False:
                    if self.count == 0:
                        control = self.compose_control(0,0,self.current_yaw,0,1)
                        self.control_pub.publish(control)
                        time.sleep(0.1)
                        self.count = 1
                    
                    control = self.compose_control(0,0,self.current_yaw,0,4)
                    self.control_pub.publish(control)
                    time.sleep(0.1)
                    # time_current = rospy.Time.now()
                    while self.complete_state ==0:
                    # and (rospy.Time.now()-time_current).to_sec()<10*60:
                        if self.rc_control == 0:
                            rospy.logwarn("rc_control == 0")
                            return
                        pass
                    if self.complete_state == 4:
                        self.in_dock_flag = True
                        self.count  = 0
                        while self.rc_control !=2:
                            if self.rc_control == 0:
                                rospy.logwarn("rc_control == 0")
                                return
                            control = self.compose_control(0,0,self.current_yaw,0,1)
              
                            self.control_pub.publish(control)
                            time.sleep(0.1)
                    else:
                        self.error = 1
                        control = self.compose_control(0,0,self.current_yaw,0,1)
                      
                        self.control_pub.publish(control)
            
            elif self.rc_control == 2:  

                # self.in_dock_flag = False
                
                if self.out_dock_flag == False:
                    # if self.count == 0:
                    control = controlData()
                    control.distance = 0
                    control.target_yaw = 0
                    control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
                    control.roller_speed = 0
                    control.robot_state = 1
                    control.header.stamp = rospy.Time.now()
                    control.header.seq = self.control_seq
                    self.control_pub.publish(control)
                    self.latitude_drone = self.latitude
                    self.longitude_drone = self.longitude
                    time.sleep(0.1)
                        # self.count = 1
                    control = controlData()
                    control.distance = 0
                    control.target_yaw = 0
                    control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
                    control.roller_speed = 0
                    control.robot_state = 3
                    self.control_pub.publish(control)
                    time.sleep(0.1)
                    time_current = rospy.Time.now()
                    while self.complete_state !=3: 
                    # and (rospy.Time.now()-time_current).to_sec()<10*60:
                        if self.rc_control == 0:
                            rospy.logwarn("rc_control == 0")
                            return
                        pass
                    if self.complete_state == 3:
                        self.out_dock_flag = True
                        self.in_dock_flag = True
                        self.docking_flag = False
                        if self.latitude_drone != 0 and self.longitude_drone != 0:
                            self.latitude_drone = self.latitude
                            self.longitude_drone = self.longitude
                        self.count = 0
                    else:
                        self.error = 1
                        
                    return
                
                if self.corner_finding_flag == False:
                    # if self.count == 0:
                    control = controlData()
                    control.distance = 0
                    control.target_yaw = 0
                    control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
                    control.roller_speed = 0
                    control.robot_state = 1
                    control.header.stamp = rospy.Time.now()
                    control.header.seq = self.control_seq
                    self.control_pub.publish(control)
                    time.sleep(0.1)
                        # self.count = 1

                    control = controlData()
                    control.distance = 0
                    control.target_yaw = 0
                    control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
                    control.roller_speed = 0
                    control.robot_state = 7
                    self.control_pub.publish(control)
                    time.sleep(0.1)
                    time_current = rospy.Time.now()
                    while self.complete_state !=7:
                        # and (rospy.Time.now()-time_current).to_sec()<10*60:
                        if self.rc_control == 0:
                            rospy.logwarn("rc_control == 0")
                            return
                        pass
                    if self.complete_state == 7:
                        self.corner_finding_flag = True
                        self.count  = 0
                        rospy.logwarn("corner_finding_flag")
                    else:
                        self.error = 1
                    return
                
                if self.auto_cleaning_flag == False:
                    # if self.count == 0:
                    control = controlData()
                    control.distance = 0
                    control.target_yaw = 0
                    control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
                    control.roller_speed = 0
                    control.robot_state = 1
                    control.header.stamp = rospy.Time.now()
                    control.header.seq = self.control_seq
                    self.control_pub.publish(control)
                    time.sleep(0.1)
                    # self.count = 1
                    
                    control = controlData()
                    control.distance = 0
                    control.target_yaw = 0
                    control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
                    control.roller_speed = 2800
                    control.robot_state = 8
                    self.control_pub.publish(control)
                    time.sleep(0.1)
                    time_current = rospy.Time.now()
                    while self.complete_state !=8:
                    # and (rospy.Time.now()-time_current).to_sec()<10*60:
                        pass
                    if self.complete_state == 8:
                        self.auto_cleaning_flag = True
                        self.count  = 0
                        while self.rc_control != 1:
                            if self.rc_control == 0:
                                rospy.logwarn("rc_control == 0")
                                return
                            control = controlData()
                            control.distance = 0
                            control.target_yaw = 0
                            control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
                            control.roller_speed = 0
                            control.robot_state = 1 
                            self.control_pub.publish(control)
                            time.sleep(0.1)
                            pass  
                    else:
                        self.error = 1
                    return
            else:
                control = self.compose_control(0,0,self.current_yaw,0,1)
                self.control_pub.publish(control)
                return
        else:
            control = self.compose_control(0,0,self.current_yaw,0,1)
            self.control_pub.publish(control)
            return

#---------------------------------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('aruco_docking_controller')
    controller = ArucoDockingController()
    rospy.spin()
