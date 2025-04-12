#!/usr/bin/env python
import rospy
import tf2_ros
import utm
import math
import numpy as np
from tf2_geometry_msgs import do_transform_pose
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped,Quaternion
from std_msgs.msg import Float64
from robot_localization.msg import INSPVAE, GPSData
from robot_control.msg import controlData
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Header

class DockingController:
    def __init__(self):
        # TF配置
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅器
        # rospy.Subscriber("/inspvae_data", INSPVAE, self.robot_gps_cb)
        # rospy.Subscriber("/gps/fix", NavSatFix , self.drone_gps_cb)
        rospy.Subscriber("camera/aruco_100/pose", PoseStamped, self.left_cb)
        rospy.Subscriber("camera/aruco_101/pose", PoseStamped, self.right_cb)
        rospy.Subscriber("camera/aruco_102/pose", PoseStamped, self.front_cb)
        
        # 发布控制指令
        self.control_pub = rospy.Publisher("/control_data", controlData, queue_size=1)
        
        # 状态参数
        self.marker_size = 0.2  # ArUco实际尺寸（米）
        self.target_distance = 0.5  # 最终对接距离（米）
        self.state = "INIT"
        self.control_rate = rospy.Rate(100)  # 10Hz控制频率

        # 存储检测到的标记信息
        self.detected_markers = {
            'left': None,
            'right': None,
            'center': None
        }

    def transform_to_base(self, pose, target_frame="base_link"):
        """将位姿转换到机器人基坐标系"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.1))
            transformed_pose = tf2_ros.do_transform_pose(pose, transform)
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"坐标转换失败: {str(e)}")
            return None

    def left_cb(self, msg): self.process_marker(msg, 'left')
    def right_cb(self, msg): self.process_marker(msg, 'right')
    def center_cb(self, msg): self.process_marker(msg, 'center')

    def process_marker(self, msg, marker_type):
        """处理检测到的ArUco标记"""
        base_pose = self.transform_to_base(msg)
        if base_pose:
            self.detected_markers[marker_type] = {
                'position': np.array([base_pose.pose.position.x, 
                                     base_pose.pose.position.y]),
                'orientation': base_pose.pose.orientation
            }
            self.update_state()

    def update_state(self):
        """状态机更新逻辑"""
        if self.detected_markers['center']:
            dist = np.linalg.norm(self.detected_markers['center']['position'])
            self.state = "FINAL_ALIGN" if dist < self.target_distance*2 else "CENTER_GUIDE"
        elif any([self.detected_markers['left'], self.detected_markers['right']]):
            self.state = "SIDE_GUIDE"
        else:
            self.state = "SEARCH"

    def calculate_control(self):
        """根据当前状态生成控制指令"""
        control = controlData()
        
        if self.state == "FINAL_ALIGN":
            # 最终精确对准阶段
            control = self.final_alignment()
            
        elif self.state == "CENTER_GUIDE":
            # 使用中间标记引导
            control = self.center_guidance()
            
        elif self.state == "SIDE_GUIDE":
            # 使用单侧标记引导
            control = self.side_guidance()
            
        else:  # SEARCH状态
            control.travel_distance = 0
            control.target_heading = 0  # 原地旋转搜索
            
        return control

    def final_alignment(self):
        """最终对准阶段控制"""
        pos = self.detected_markers['center']['position']
        q = self.detected_markers['center']['orientation']
        
        # 计算机器人到标记的向量
        dx = pos[0]
        dy = pos[1]
        
        # 计算目标航向（标记坐标系到机器人坐标系）
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        target_yaw = yaw + math.pi  # 需要面向标记背面
        
        # 计算横向偏差
        lateral_error = dy
        
        return controlData(
            distance=dx - self.target_distance,
            yaw=target_yaw,
            # lateral_error=lateral_error
        )

    def center_guidance(self):
        """中间标记引导阶段"""
        pos = self.detected_markers['center']['position']
        dx = pos[0]
        dy = pos[1]
        
        # 航向角计算（指向标记中心）
        target_heading = math.atan2(dy, dx)
        
        # 保持前向运动
        distance = math.sqrt(dx**2 + dy**2) - self.target_distance
        
        return controlData(
            distance=distance,
            target_yaw=target_heading
        )

    def side_guidance(self):
        """单侧标记引导逻辑"""
        if self.detected_markers['left'] and self.detected_markers['right']:
            # 同时看到左右标记
            left_pos = self.detected_markers['left']['position']
            right_pos = self.detected_markers['right']['position']
            mid_point = (left_pos + right_pos) / 2
            return self.center_guidance(mid_point)
            
        elif self.detected_markers['left']:
            # 仅左标记可见
            return self.estimate_center('left', rightward_offset=0.5)
            
        elif self.detected_markers['right']:
            # 仅右标记可见
            return self.estimate_center('right', leftward_offset=0.5)

    def estimate_center(self, side, offset):
        """根据单侧标记估计中心位置"""
        marker_pos = self.detected_markers[side]['position']
        # 根据已知布局估算中心位置（假设标记间距1米）
        estimated_center = marker_pos.copy()
        if side == 'left':
            estimated_center[1] -= offset  # 向右偏移
        else:
            estimated_center[1] += offset  # 向左偏移
            
        dx = estimated_center[0]
        dy = estimated_center[1]
        
        return controlData(
            travel_distance=math.sqrt(dx**2 + dy**2) - self.target_distance,
            target_heading=math.atan2(dy, dx)
        )

    def run(self):
        while not rospy.is_shutdown():
            self.update_state()
            control = self.calculate_control()
            self.control_pub.publish(control)
            self.control_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('docking_node')
    controller = DockingController()
    controller.run()
       