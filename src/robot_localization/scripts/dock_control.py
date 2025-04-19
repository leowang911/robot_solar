#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import tf2_ros
import numpy as np
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion
import tf.transformations
from geographic_msgs.msg import GeoPoint
from geodesy import utm
from geometry_msgs.msg import PoseStamped, Twist, PoseArray,PointStamped,Quaternion
from robot_control.msg import controlData 
from robot_localization.msg import INSPVAE,baseStatus, GPSData
from std_msgs.msg import Int16, Int32,Header
from sensor_msgs.msg import Image
import time
class ArucoDockingController:
    def __init__(self):
        
        self.info = True
        self.logprint=False
        # 坐标系参数
        self.marker_spacing = rospy.get_param('~marker_spacing', 1.0)  # 左右标记间距（米）
        self.marker_side_spacing   = rospy.get_param('~marker_side_spacing', 0.78)  # 中间标记与侧标记间距（米）
        self.stop_distance = rospy.get_param('~stop_distance', 1.0)  # 中间标记前停止距离
        self.stop_distance_threshold = rospy.get_param('stop_distance_threshold', 0.1)  # 停止距离阈值
        self.target_distance = 1 # 目标距离（米）
        self.align_threshold = math.radians(1)  # 航向对准阈值
        self.current_yaw = 0 # 当前航向角
        self.target_yaw = 0# 目标航向角
        self.latitude = 30.32098151262
        self.longitude = 120.07004749195
        self.latitude_drone = 30.32098151262
        # self.longitude_drone = -74.123339
        self.longitude_drone = 120.07004749195
        self.yaw_drone = 0.0
        self.speed = 0.0
        self.distance2drone = 0.0
        self.yaw2drone = 0.0
        self.depth_image = None
        self.back=False
        self.refine_align=False
        self.align_num=False
        self.lock_current=False
        self.lock_refine=False
        # TF配置
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.flag_count = 0
        # 新增数据有效期参数（单位：秒）
        self.data_expiry = 5  # 0.5秒未更新的数据视为失效
        self.marker_time = {'left': None, 'right': None, 'center': None, 'center_left': None, 'center_right': None}
        self.valid_center_markers = []
        self.center_side_offset = [ 0.37608,-0.01905,-0.42418]
        self.complete_state = 0
        self.first_look_flag = False

        # 存储检测数据（基坐标系）
        self.markers = {
            'left': None, 
            'right': None,
            'center': None,
            'center_left': None,
            'center_right': None
        }


        # 状态变量
        self.state = "INIT"
        self.state_prev = "SEARCH"
        self.estimated_center = None
        self.current_target = {
            'position': np.array([0.0, 0.0, 0.0]),
            'yaw': 0.0,
            'center': np.array([0.0, 0.0, 0.0]),
        }


        #  # 新增滤波参数
        # self.filter_enabled = True          # 滤波开关
        # self.filter_time_constant = 0.2     # 低通滤波时间常数（秒）
        # self.ema_alpha = 0.3                # EMA平滑系数（0-1）
        
        # # 滤波状态变量
        # self.filtered_yaw = 0.0             # 滤波后航向角
        # self.last_filter_time = None        # 上次滤波时间
        


        # 订阅器
        rospy.Subscriber("/inspvae_data", INSPVAE, self.inspvae_cb)
        rospy.Subscriber("/base_status", baseStatus, self.base_cb)
        rospy.Subscriber("/gps/raw", GPSData, self.drone_gps_cb)
        rospy.Subscriber("/camera/aruco_100/pixel", PointStamped, self.left_cb)
        rospy.Subscriber("/camera/aruco_101/pixel", PointStamped, self.right_cb)
        rospy.Subscriber("/camera/aruco_102/pixel", PointStamped, self.center_cb)
        rospy.Subscriber("/camera/aruco_103/pixel", PointStamped, self.center_left_cb)
        rospy.Subscriber("/camera/aruco_104/pixel", PointStamped, self.center_right_cb)

        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_cb)
        
        # rospy.Subscriber("/virtual_marker_102/pose", PoseStamped, self.center_cb)
        # rospy.Subscriber("/virtual_markers", PoseArray, self.markers_cb)
        
        # 发布器
        self.control_pub = rospy.Publisher("/control_data", controlData, queue_size=1)
        self.pose1_pub = rospy.Publisher("/marker_pose1", PoseStamped, queue_size=1)
        self.pose2_pub = rospy.Publisher("/marker_pose2", PoseStamped, queue_size=1)
        self.pose3_pub = rospy.Publisher("/marker_pose3", PoseStamped, queue_size=1)
        
    
        self.control_seq = 0
        
        #rospy.Timer(rospy.Duration(0.01), self.control_loop)
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def depth_cb(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint16 if msg.is_bigendian else '<u2')
        self.depth_image = data.reshape(msg.height, msg.width)



    # sim only
    def markers_cb(self, msg):
        """处理虚拟标记数据"""
        for i, pose in enumerate(msg.poses):
            marker_id = 100 + i  # ID对应100,101,102
            ps = PoseStamped()
            ps.pose = pose
            ps.header = msg.header
            self.process_marker(ps, ['left', 'right', 'center'][i])

    def pixel_to_point(self,uvz, fx, fy, cx, cy):
        """将单个像素坐标+深度转换为三维坐标"""
        u,v,z=uvz

        # 过滤无效深度值
        valid_mask = (z > 0)
        u = u[valid_mask]
        v = v[valid_mask]
        z = z[valid_mask]


        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        return np.array([X, Y, z])

    def transform_to_base(self, pose):
    # """将位姿转换到机器人基坐标系"""
        try:
            # 获取坐标系变换关系
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                pose.header.frame_id,
                pose.header.stamp,  # 使用原始消息的时间戳
                rospy.Duration(0.1)
            )
            transformed = do_transform_pose(pose, transform)
            
            # 直接使用变换后的坐标（无需手动调整）
            return {
                'position': np.array([
                    transformed.pose.position.x,
                    transformed.pose.position.y,
                    transformed.pose.position.z
                ]),
                'orientation': transformed.pose.orientation,
                'pixel': np.array([pose.pixel.x, pose.pixel.y])
            }
        except Exception as e:
            rospy.logwarn(f"坐标转换失败: {str(e)}")
            return None

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
        # 处理GPS数据
        self.latitude_drone = msg.latitude
        self.longitude_drone = msg.longitude
        self.yaw_drone = msg.yaw

    def inspvae_cb(self, msg):
        # self.latitude = msg.latitude
        # self.longitude = msg.longitude
        self.current_yaw = math.radians(msg.yaw)

    def left_cb(self, msg): self.process_marker(msg, 'left')
    def right_cb(self, msg): self.process_marker(msg, 'right')
    def center_cb(self, msg): self.process_marker(msg, 'center')
    def center_left_cb(self, msg): self.process_marker(msg, 'center_left')
    def center_right_cb(self, msg): self.process_marker(msg, 'center_right')



    def process_marker(self, msg, marker_type):
        """处理ArUco检测数据（增加时间戳）"""
        # base_data = self.transform_to_base(msg)
        # if base_data:
        self.markers[marker_type] = msg.point
        self.marker_time[marker_type] = msg.header.stamp # 记录时间戳
        # self.markers_pixel[marker_type] = msg.pose.pixel
        # 记录更新时间
        self.update_state()

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

    def update_state(self):
        """状态机更新（增加数据有效性检查）"""
        self.check_data_expiry()  # 先执行数据清理
        self.valid_center_markers = []
        valid_target = []
        current_target = {
            'position': np.array([0.0, 0.0, 0.0]),
            'yaw': 0.0,
            'center': np.array([0.0, 0.0, 0.0]),
        }

        # 检查是否有有效数据
        valid_left = self.markers['left'] is not None
        valid_right = self.markers['right'] is not None
        valid_center = self.markers['center'] is not None
        valid_center_left = self.markers['center_left'] is not None
        valid_center_right = self.markers['center_right'] is not None
        # rospy.loginfo(f"有效数据: left={valid_left}, right={valid_right}, center={valid_center}")

        # 状态优先级更新
        # if self.state == "FINAL_DOCKING":
        #     self.state = "FINAL_DOCKING"
        # else:

        if self.markers['center'] is not None:
            # self.state = "FINAL_APPROACH"
            
            self.valid_center_markers.append(self.markers['center'])
            # if valid_center_left:
            #     self.valid_center_markers.append(self.markers['center_left'])
            # if valid_center_right:
            #     self.valid_center_markers.append(self.markers['center_right'])
            valid_target.append(self.calculate_center_target())
            #rospy.loginfo(f"center: {valid_target}")

    

        if self.markers['center_left'] is not None:
            valid_target.append(self.calculate_center_side_target('center_left'))
            # rospy.loginfo(f"left: {valid_target}")

        
        if self.markers['center_right'] is not None:
            valid_target.append(self.calculate_center_side_target('center_right'))
            # rospy.loginfo(f"right: {valid_target}")

        # for marker_type in ['left', 'right', 'center', 'center_left', 'center_right']:
        #     rospy.loginfo(f"{marker_type} marker_time: {self.marker_time[marker_type]}")

        if self.markers['left'] or self.markers['right'] or self.markers['center'] or self.markers['center_left'] or self.markers['center_right']:
            self.state = "APPROACHING"
            if self.first_look_flag == False:
                self.first_look_flag = True
                control = controlData()
                control.distance = 0 
                # control.target_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
                # control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
                control.target_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
                control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
                control.roller_speed = 0
                control.robot_state = 1
                self.control_pub.publish(control)
                time.sleep(0.1)
                    
        else:
            self.state = "SEARCH"
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
                        
            return    
        

        # if valid_left:
        #     # self.current_target = self.estimate_center('left')
        # if valid_right:
            # self.current_target = self.estimate_center('right')

            
        # elif valid_left:
        #     self.state = "ESTIMATED_APPROACH"
        #     self.current_target = self.estimate_center('left')
        # elif valid_right:
        #     self.state = "ESTIMATED_APPROACH"
        #     self.current_target = self.estimate_center('right')
        

        # rospy.loginfo(f"当前状态: {self.state}")



    def calculate_center_side_target(self, side):
        """计算中间标记前的目标点（基于单侧标记）"""
        marker = self.markers[side]
        # pos = marker['position']
        # rot = marker['orientation']
        
        pose_stamped=self.get_rot(self.markers[side])
        pose = pose_stamped.pose
        if side == 'center_left':
            self.pose2_pub.publish(pose_stamped)
        else:
            self.pose3_pub.publish(pose_stamped)
            
        pos=np.array([pose.position.x,pose.position.y,pose.position.z])
        rot=pose.orientation
        R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        sign = 1 if side == 'center_right' else -1
        # 计算中间位置 * sign
        offset = self.marker_side_spacing/2 *sign  +0.05
        self.pos_target = R@np.array([-offset,0, self.stop_distance]) + pos
        pos_center = R@np.array([-offset,0, 0]) + pos
        # rospy.loginfo(f"pos: {pos}")
        # rospy.loginfo(f"self.pos_target : {self.pos_target }")

        return {
            'position': self.pos_target,
            'yaw': self.get_marker_yaw(self.pos_target),
            'center': pos_center,
            # 'yaw': np.arctan2(marker['position'][1], marker['position'][0]) + np.pi/2
        }
    def fit_plane_to_points(self,points):
        """
        用NumPy拟合点云所在平面
        :param points: Nx3的NumPy数组，输入点云
        :return: (A, B, C, D) 平面方程系数，法向量为(A, B, C)
        """
        # 1. 计算质心
        centroid = np.mean(points, axis=0)
        
        # 2. 去中心化
        centered = points - centroid
        
        # 3. 计算协方差矩阵
        cov_matrix = np.cov(centered, rowvar=False)  # 输入为Nx3，rowvar=False表示列代表变量
        
        # 4. 特征分解，求最小特征值对应的特征向量（法向量）
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        normal = eigenvectors[:, np.argmin(eigenvalues)]  # 最小特征值对应的特征向量
        
        # 5. 计算D：Ax0 + By0 + Cz0 + D = 0 => D = -(A*x0 + B*y0 + C*z0)
        A, B, C = normal
        D = -np.dot(normal, centroid)
        
        return A, B, C, D,centroid
    
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
    def get_pose(self, a, b, c):
        yaxis=np.array([0,-1.0,0.0])
        zaxis=np.array([a,b,c])
        # 计算v2在e1方向的投影
        proj = np.dot(zaxis, yaxis) * yaxis
        
        # 计算正交分量并归一化
        u2 = zaxis - proj
        norm_u2 = np.linalg.norm(u2)
        if norm_u2 < 1e-10:
            raise ValueError("Vectors are parallel or invalid input")
        zaxis = u2 / norm_u2
        
        if zaxis[2]>0:
            zaxis=-zaxis
        xaxis=np.cross(yaxis,zaxis)
        qua=self.axes_to_quaternion(xaxis, yaxis, zaxis)
        return qua
    def get_rot(self,pixel):
        u=int(pixel.x)
        v=int(pixel.y)
        u_ax=np.arange(u-15,u+15)
        v_ax=np.arange(v-15,v+15)
        if self.depth_image is not None:
            pt=[]
            for i in u_ax:
                for j in v_ax:
                    if self.depth_image[j,i] is not None:
                        if abs(self.depth_image[j,i])<2000.0:
                            pt.append([i,j,self.depth_image[j,i]/1000.0])
            try:
                if len(pt)>10:
                    pt=np.array(pt).T
                    fx,fy,cx,cy=[612.3629150390625, 637.8858032226562, 612.5785522460938, 362.7610168457031]
                    point=self.pixel_to_point(pt, fx,fy,cx,cy)
                    a,b,c,d,centp=self.fit_plane_to_points(point.T)
                    pose_q= self.get_pose(a,b,c)
                else:
                    centp=np.array([0,0,0.0])
                    pose_q= self.get_pose(0,0,-1)    
            except Exception as e:  
                #rospy.logwarn(f"点云拟合失败: {str(e)}")
                centp=np.array([0,0,0.0])
                pose_q= self.get_pose(0,0,-1)       
        else:
            centp=np.array([0,0,0.3])
            pose_q= self.get_pose(0,0,-1)
        pose= PoseStamped()
        pose.header.frame_id = "camera_link"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = centp[0]
        pose.pose.position.y = centp[1]
        pose.pose.position.z = centp[2]
        pose.pose.orientation = pose_q
        transform = self.tf_buffer.lookup_transform(
                'base_link',
                pose.header.frame_id,
                pose.header.stamp,  # 使用原始消息的时间戳
                rospy.Duration(0.1)
            )
        transformed = do_transform_pose(pose, transform)
        return transformed
    def calculate_center_target(self):
        """计算中间标记前的目标点"""

        # if self.current_target is None:
        #     control = controlData()
        #     control.distance = 0
        #     control.target_yaw = 0
        #     control.robot_state = 1
        #     self.control_pub.publish(control)
        #     return

        # marker_distance = math.sqrt(self.markers['center']['position'][0]**2 + self.markers['center']['position'][1]**2)
        #pos = self.markers['center']['position']
        #rot = self.markers['orientation']
        pose_stamped=self.get_rot(self.markers['center'])
        pose = pose_stamped.pose
        self.pose1_pub.publish(pose_stamped)
        pos=np.array([pose.position.x,pose.position.y,pose.position.z])
        rot=pose.orientation
        # sum_q = np.zeros(4)
        # for p in self.valid_center_markers:
        #     q = p['orientation']
        #     sum_q += np.array([q.x, q.y, q.z, q.w])
        # norm = np.linalg.norm(sum_q)
        # if norm < 1e-6:
        #     avg_q = np.array([0.0, 0.0, 0.0, 1.0])  # 单位四元数
        # else:
        #     avg_q = sum_q / norm
        
        # rot = avg_q


        # rot = self.markers['center']['orientation']

        # # # 大于目标距离时，先走到目标点前1m
        # if abs(marker_distance - self.stop_distance) > self.stop_distance_threshold:

        R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        self.pos_target = R@[-self.center_side_offset[1],0 , self.stop_distance] + pos

        # else:
        #     # pos = self.markers['center']['position']
        #     # rot = self.markers['center']['orientation']
        # R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        pos =  R@[-0.05,0 ,0] + pos

        #     self.pos_target = [0,0,0]

        # if self.get_marker_yaw(self.markers['center']) is None:
        #         rospy.logwarn("无法获取航向角")
        #         return None
        
        
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
        self.pos_target = R@[-sign*self.stop_distance,0, offset] + pos
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
        angle= (math.degrees(yaw)*100) + math.degrees(current_yaw)*100
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
        # 航向角（北向为0，北偏东为正0-360）
        # 计算无人机相对于基坐标系的坐标
        self.distance2drone = math.sqrt(easting_diff**2 + northing_diff**2)
        self.yaw2drone = math.atan2(easting_diff, northing_diff)
        # rospy.loginfo(f'gps distance:{self.distance2drone} yaw: {self.yaw2drone}' )

    def search(self):
        #找不到旋转180
        if self.distance2drone > 1 or self.distance2drone <=0.1:
            control = controlData()
            control.distance = 0 
            # control.target_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            # control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            control.target_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, np.pi/10)
            control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            control.roller_speed = 0
            control.robot_state = 1
            self.control_pub.publish(control)
            time.sleep(0.01)
            control.robot_state = 2
            self.control_pub.publish(control)
            time.sleep(0.5)

            self.control_seq += 1
        else:
            control = controlData()
            control.distance = -200 
            # control.target_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            # control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            control.target_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
            control.roller_speed = 0
            control.robot_state = 2
            self.control_pub.publish(control)
            self.control_seq += 1
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
    def direct_back(self):

        distance=-0.2
        # theta1=math.atan(abs(prepoint[1]/prepoint[0]))

        # if prepoint[1]>0:
        #     theta1=-theta1
        theta1 = theta2=   0
        return distance,theta1,theta2


    def get_step2_robot_pose(self,):
        
        v1=self.current_target['position'][:2]
        v2=self.current_target['center'][:2]
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


    def control_loop(self, event):
        if self.markers['left'] or self.markers['right'] or self.markers['center'] or self.markers['center_left'] or self.markers['center_right']:
            self.state = "APPROACHING"
            pass
        else:
            self.update_state()
        
        
        """主控制循环"""
        control = controlData()
        control.distance = 0
        control.target_yaw = 0
        control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
        control.roller_speed = 0
        #control.robot_state = 1
        # rospy.loginfo(f"当前状态: {self.state}")

    

        #计算gps距离
        gps_calculation = self.gps_calculation(self.latitude, self.longitude, self.latitude_drone, self.longitude_drone)
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
            if self.complete_state==1:
                control.robot_state = 1
                self.control_pub.publish(control)
                control.robot_state = 2 
                time.sleep(0.01)
            self.control_pub.publish(control)
        
            self.control_seq += 1


        else: #gps距离小于2米,通过aruco数据导航

            rospy.loginfo(f'state {self.state}')
            if self.state == "SEARCH":
                 rospy.loginfo(f'SEARCH******************* {self.state}')
                 control = self.search()
                 return
            if self.current_target:
                self.lock_current=True
                current_pos = np.array([0, 0])  # 基坐标系原点
                target_vec = self.current_target['position'][:2] - current_pos
                rospy.loginfo(f'target_vec is %%%%%%%%%%% {target_vec}')
                

            # 计算当前状态,行走到目标点前1m
                if self.refine_align==False:
                    #粗定位
                    if np.linalg.norm(target_vec) >0.6:
                        self.align_num=False

                    if np.linalg.norm(target_vec) > self.stop_distance_threshold and self.align_num==False:

                        if target_vec[0]>0:
                            self.target_distance = np.linalg.norm(target_vec) 
                            self.target_distance=np.clip(self.target_distance,0,0.3)
                            self.target_yaw = math.atan2(target_vec[1], target_vec[0])
                            self.target_yaw =np.clip(self.target_yaw,-0.2,0.2)
                        else:
                            self.target_yaw = 0
                            control.distance = -100
                            control.target_yaw = self.yaw_to_target_yaw_angle(0,self.current_yaw)
                            control.robot_state = 2
                            control.header.stamp = rospy.Time.now()
                            control.robot_state = 1
                            self.control_pub.publish(control)
                            time.sleep(0.05)
                            control.header.stamp = rospy.Time.now()
                            control.robot_state = 2 
                            self.control_pub.publish(control)
                            self.control_seq += 1
                            time.sleep(1.0)
                            return 
                            
                        if  np.linalg.norm(target_vec)<0.1:
                            self.target_yaw=0
                        

                        control.distance = int(self.target_distance*1000)
                        control.target_yaw = self.yaw_to_target_yaw_angle(self.target_yaw,self.current_yaw)
                        control.robot_state = 2

                        # 发布控制指令
                        control.header.stamp = rospy.Time.now()
                        control.header.seq = self.control_seq
                        self.state_prev = self.state
                        rospy.loginfo(f'state: {control.robot_state}')
                        if self.complete_state==1:
                            control.robot_state = 1
                            control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)
                            control.robot_state = 2 
                            control.header.stamp = rospy.Time.now()
                            time.sleep(0.01)
                        self.control_pub.publish(control)
                    
                        self.control_seq += 1
                        # rospy.loginfo(f"")
                        rospy.loginfo(f"未到达目标位置: {self.current_target['position']},{self.get_marker_yaw(self.current_target['position'])}")
                        rospy.loginfo(f"complete_state: {self.complete_state}")
                        
                    else:
                        if self.align_num==False:
                            control.robot_state = 1
                            control.header.stamp = rospy.Time.now()
                            control.header.seq = self.control_seq

                            self.control_pub.publish(control)
                            time.sleep(0.05)
                            control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)
                            control.robot_state = 2

                            self.control_seq += 1
                            self.align_num=True
                            return
                    
                if self.align_num==True:
                        self.state = "FINAL_DOCKING"
                        rospy.loginfo(f"到达目标位置: {self.current_target['center']},{self.get_marker_yaw(self.current_target['center'])}")
                        rospy.loginfo(f"到达目标位置__yaw: {self.current_yaw}")
                        if abs(self.get_marker_yaw(self.current_target['center'])) < 0.015:
                                rospy.logwarn(f"完成对正 {target_vec[0]} {target_vec[1]}")
                                rospy.logwarn("robot start is 1")
                                self.refine_align=True
                                control.robot_state = 1
                                control.header.stamp = rospy.Time.now()

                                self.control_pub.publish(control)
                                time.sleep(0.05)
                                control.header.stamp = rospy.Time.now()
                                self.control_pub.publish(control)
    
                                control.robot_state = 2

                                self.control_seq += 1
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

                            rospy.loginfo(f"taget_yal:{ control.target_yaw}, ￥￥￥￥￥￥curent_yaw: {self.current_yaw}")

                            # 发布控制指令
                            control.header.stamp = rospy.Time.now()
                            control.header.seq = self.control_seq
                            self.state_prev = self.state
                            rospy.loginfo(f'state: {control.robot_state}')
                            if self.complete_state==1:
                                control.robot_state = 1
                                control.header.stamp = rospy.Time.now()
                                self.control_pub.publish(control)
                                control.robot_state = 2 
                                time.sleep(0.01)
                                control.header.stamp = rospy.Time.now() 
                            self.control_pub.publish(control)
                        
                            self.control_seq += 1
                            self.refine_align=False

                            
                if self.refine_align==True:
                            #精确对齐
                            if np.linalg.norm(target_vec) >1.2:
                                self.refine_align=False 
                                return 

                            if np.linalg.norm(target_vec) <0.10 and abs(target_vec[1])<0.02:
                                control.robot_state = 4
                                rospy.loginfo(f'GOOD *****************************************************start final docking')

                                control.header.stamp = rospy.Time.now()
                                self.stop_distance_threshold=0.1
                                self.control_pub.publish(control)
                                self.control_seq += 1 
                                time.sleep(1000)                               
                                return 
                            

                            #step1 
                            #get current robot pose

                            control.distance = 0
                            control.target_yaw = 0                            
                            control.robot_state = 1
                            control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)
                            time.sleep(0.1)
                            self.complete_state = 0
                            self.lock_refine=True
                            #d1,yaw1,yaw2=self.get_pre_robot_pose()
                            d1,yaw1,yaw2=self.direct_back()
                            rospy.loginfo(f'robot pose1: {d1} {yaw1} {yaw2}')
                            control.distance = int(d1*1000)
                            control.target_yaw = self.yaw_to_target_yaw_angle(yaw1,self.current_yaw)
                            control.robot_state = 2
                            control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)
                            time.sleep(0.5)
                            rospy.loginfo(f'等待回退结束 ')
                            while self.complete_state != 1:
                                

                                # time.sleep(0.1)
                                pass
                            rospy.loginfo(f'成功回退！！ ')
                            #执行结束
                            control.distance = 0
                            control.target_yaw = 0                            
                            control.robot_state = 1
                            control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)
                            time.sleep(0.05)
                            control.distance = 0
                            control.target_yaw = 0                            
                            control.robot_state = 1
                            control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)
                            time.sleep(0.05)

                            self.complete_state = 0
                            control.distance = 0
                            control.target_yaw = self.yaw_to_target_yaw_angle(yaw2,self.current_yaw)                            
                            control.robot_state = 2
                            control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)
                            time.sleep(0.1)
                            rospy.loginfo(f'等待回正结束 ')     
                            while self.complete_state != 1:
                                
                                # time.sleep(0.1)
                                pass
                            rospy.loginfo(f'step1 成功回正！ ')
                            self.lock_refine=False
                            #执行结束
                            time.sleep(2.0)
                            #step2
                            control.distance = 0
                            control.target_yaw = 0                            
                            control.robot_state = 1
                            control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)
                            time.sleep(0.05)
                            control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)


                            #self.complete_state = 0
                            self.lock_refine=True
                            d1,yaw1,yaw2=self.get_step2_robot_pose()
                            rospy.loginfo(f'robot pose22: {d1} {yaw1} {yaw2}')
                            control.distance = int(d1*1000)
                            control.target_yaw = self.yaw_to_target_yaw_angle(yaw1,self.current_yaw)
                            control.robot_state = 2
                            control.header.stamp = rospy.Time.now()
                            self.control_pub.publish(control)
                            time.sleep(1.0)
                            rospy.loginfo(f'等待前进结束 ')
                            while self.complete_state != 1:
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
                            while self.complete_state != 1:
                               
                                pass
                            rospy.loginfo(f'step2 成功回正！！')


                            #对齐
                    




                            self.lock_refine=False
                            time.sleep(0.5)
                            #执行结束


                            return
                self.lock_current=False




if __name__ == '__main__':
    rospy.init_node('aruco_docking_controller')
    controller = ArucoDockingController()
    rospy.spin()