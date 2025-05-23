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
from geometry_msgs.msg import PoseStamped, Twist, PoseArray
from robot_control.msg import controlData 
from robot_localization.msg import INSPVAE,baseStatus, GPSData
from std_msgs.msg import Int16, Int32,Header

class ArucoDockingController:
    def __init__(self):
        
        self.info = True
        self.logprint=False
        # 坐标系参数
        self.marker_spacing = rospy.get_param('~marker_spacing', 1.0)  # 左右标记间距（米）
        self.marker_side_spacing   = rospy.get_param('~marker_side_spacing', 0.8)  # 中间标记与侧标记间距（米）
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
        
        
        # TF配置
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 新增数据有效期参数（单位：秒）
        self.data_expiry = 1  # 0.5秒未更新的数据视为失效
        self.marker_time = {'left': None, 'right': None, 'center': None, 'center_left': None, 'center_right': None}
        self.valid_center_markers = []
        
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
        rospy.Subscriber("/camera/aruco_100/pose", PoseStamped, self.left_cb)
        rospy.Subscriber("/camera/aruco_101/pose", PoseStamped, self.right_cb)
        rospy.Subscriber("/camera/aruco_102/pose", PoseStamped, self.center_cb)
        rospy.Subscriber("/camera/aruco_103/pose", PoseStamped, self.center_left_cb)
        rospy.Subscriber("/camera/aruco_104/pose", PoseStamped, self.center_right_cb)
        # rospy.Subscriber("/virtual_marker_102/pose", PoseStamped, self.center_cb)
        # rospy.Subscriber("/virtual_markers", PoseArray, self.markers_cb)
        
        # 发布器
        self.control_pub = rospy.Publisher("/control_data", controlData, queue_size=1)
        
        # 状态变量
        self.state = "SEARCH"
        self.state_prev = "SEARCH"
        self.estimated_center = None
        self.current_target = {
            'position': np.array([0.0, 0.0, 0.0]),
            'yaw': 0.0,
            'center': np.array([0.0, 0.0, 0.0]),
        }



        self.control_seq = 0
        
        # 存储检测数据（基坐标系）
        self.markers = {
            'left': None, 
            'right': None,
            'center': None,
            'center_left': None,
            'center_right': None
        }


        #rospy.Timer(rospy.Duration(0.01), self.control_loop)
        rospy.Timer(rospy.Duration(0.10), self.control_loop)

    # sim only
    def markers_cb(self, msg):
        """处理虚拟标记数据"""
        for i, pose in enumerate(msg.poses):
            marker_id = 100 + i  # ID对应100,101,102
            ps = PoseStamped()
            ps.pose = pose
            ps.header = msg.header
            self.process_marker(ps, ['left', 'right', 'center'][i])

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
                'orientation': transformed.pose.orientation
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
        base_data = self.transform_to_base(msg)
        if base_data:
            self.markers[marker_type] = base_data
            self.marker_time[marker_type] = msg.header.stamp # 记录时间戳
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
                rospy.logdebug(f"清除过期标记数据: {marker_type}")

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
        if self.state == "FINAL_DOCKING":
            self.state = "FINAL_DOCKING"
        else:
            if valid_center :
                self.state = "FINAL_APPROACH"
                
                self.valid_center_markers.append(self.markers['center'])
                # if valid_center_left:
                #     self.valid_center_markers.append(self.markers['center_left'])
                # if valid_center_right:
                #     self.valid_center_markers.append(self.markers['center_right'])
                valid_target.append(self.calculate_center_target())
                rospy.loginfo(f"center: {valid_target}")

        

            if valid_center_left:
                valid_target.append(self.calculate_center_side_target('center_left'))
                rospy.loginfo(f"left: {valid_target}")

            
            if valid_center_right:
                valid_target.append(self.calculate_center_side_target('center_right'))
                rospy.loginfo(f"right: {valid_target}")


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
            

            if valid_left:
                self.current_target = self.estimate_center('left')
            if valid_right:
                self.current_target = self.estimate_center('right')

                
            # elif valid_left:
            #     self.state = "ESTIMATED_APPROACH"
            #     self.current_target = self.estimate_center('left')
            # elif valid_right:
            #     self.state = "ESTIMATED_APPROACH"
            #     self.current_target = self.estimate_center('right')
            # else:
            #     self.state = "SEARCH"
            #     self.current_target = None  # 清空目标

        # rospy.loginfo(f"当前状态: {self.state}")



    def calculate_center_side_target(self, side):
        """计算中间标记前的目标点（基于单侧标记）"""
        marker = self.markers[side]
        pos = marker['position']
        rot = marker['orientation']
        R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        sign = 1 if side == 'center_right' else -1
        # 计算中间位置 * sign
        offset = self.marker_side_spacing/2 *sign
        self.pos_target = R.T@np.array([-offset,0, self.stop_distance]) + pos
        pos_center = R.T@np.array([-offset,0, 0]) + pos
        rospy.loginfo(f"pos: {pos}")
        rospy.loginfo(f"self.pos_target : {self.pos_target }")

        return {
            'position': self.pos_target,
            'yaw': self.get_marker_yaw(self.pos_target),
            'center': pos_center,
            # 'yaw': np.arctan2(marker['position'][1], marker['position'][0]) + np.pi/2
        }


    def calculate_center_target(self):
        """计算中间标记前的目标点"""

        # if self.current_target is None:
        #     control = controlData()
        #     control.distance = 0
        #     control.target_yaw = 0
        #     control.robot_state = 1
        #     self.control_pub.publish(control)
        #     return

        marker_distance = math.sqrt(self.markers['center']['position'][0]**2 + self.markers['center']['position'][1]**2)
        pos = self.markers['center']['position']
        rot = self.markers['orientation']

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
        self.pos_target = R@[0, 0, self.stop_distance] + pos

        # else:
        #     # pos = self.markers['center']['position']
        #     # rot = self.markers['center']['orientation']
        # R = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        pos_center =  pos
        #     self.pos_target = [0,0,0]

        # if self.get_marker_yaw(self.markers['center']) is None:
        #         rospy.logwarn("无法获取航向角")
        #         return None
        
        
        return {
            'position': self.pos_target,
            'yaw': self.get_marker_yaw(self.pos_target),
            'center': pos_center,
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
        pos = self.markers[side]['position']
        rot = self.markers[side]['orientation']
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
        rospy.loginfo(f"current_yaw: {self.current_yaw}")
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

    def control_loop(self, event):
        """主控制循环"""
        control = controlData()
        control.distance = 0
        control.target_yaw = 0
        control.yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
        control.roller_speed = 0
        control.robot_state = 1
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
        else: #gps距离小于2米,通过aruco数据导航
            if self.current_target:
            # 计算当前状态,行走到目标点前1m
                current_pos = np.array([0, 0])  # 基坐标系原点
                target_vec = self.current_target['position'][:2] - current_pos
                self.target_distance = np.linalg.norm(target_vec) 
                self.target_yaw = math.atan2(target_vec[1], target_vec[0])
                # rospy.loginfo(f"\n yaw: {target_yaw }\n distance: {distance}\n yaw_error: {yaw_error}\n state: {self.state}\n")
                # 状态处理

                if self.target_distance > self.stop_distance_threshold:
                    rospy.loginfo(f"target_distance: {self.target_distance}")
                    # if abs(yaw_error) > self.align_threshold:
                    #     # 航向调整阶段
                    control.distance = int(self.target_distance*1000)
                    control.target_yaw = self.yaw_to_target_yaw_angle(self.current_target['yaw'],self.current_yaw)
                    control.robot_state = 2
                    # else:
                    #     # 直线移动阶段
                    #     control.distance = int(distance*1000)
                    #     control.target_yaw = self.yaw_to_target_yaw_angle(target_yaw)
                else:
                    # 到达目标位置
                    self.state = "FINAL_DOCKING"

                    control.distance = 0
                    control.target_yaw = self.yaw_to_target_yaw_angle(self.get_marker_yaw(self.current_target['center']),self.current_yaw)
                    control.robot_state = 2
                                        
                    if self.state_prev == "FINAL_APPROACH":
                        control.robot_state = 1

                    # if self.state == "FINAL_APPROACH":
                    #     # if self.state_prev
                        

                    #     control.distance = 0
                    #     control.target_yaw = self.yaw_to_target_yaw_angle(self.current_target['yaw'],self.current_yaw)
                    #     control.robot_state = 2
                    # else:
                    #     # 进入最终对接
                    #     self.state = "FINAL_DOCKING"
                    #     control.robot_state = 1

            # else:
            #     control.distance = 0
            #     control.target_yaw = 0
            #     control.robot_state = 1

        # 查看所有变量
        # if self.info:
            rospy.loginfo(f"当前状态: {self.pos_target}")
            # rospy.loginfo(f"当前航向角: {self.current_yaw}")
            # rospy.loginfo(f"目标航向角: {self.target_yaw}")
            # rospy.loginfo(f"目标距离: {self.target_distance}")
            # rospy.loginfo(f"当前速度: {self.speed}")
            # rospy.loginfo(f"无人机坐标: {self.latitude_drone}, {self.longitude_drone}")
            # rospy.loginfo(f"基坐标系坐标: {self.latitude}, {self.longitude}")
            # rospy.loginfo(f"无人机航向角: {self.yaw_drone}")
            # rospy.loginfo(f"无人机与基坐标系距离: {self.distance2drone}")  
            # rospy.loginfo(f"无人机航向角: {self.yaw2drone}")

        # 发布控制指令
        control.header.stamp = rospy.Time.now()
        control.header.seq = self.control_seq
        self.state_prev = self.state
        self.control_pub.publish(control)
        self.control_seq += 1

if __name__ == '__main__':
    rospy.init_node('aruco_docking_controller')
    controller = ArucoDockingController()
    rospy.spin()