#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import tf.transformations
from geometry_msgs.msg import TransformStamped
from robot_control.msg import controlData

class VirtualActuator:
    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 初始化位姿 [x, y, yaw]
        self.position = np.array([0.0, 0.0, 0.0])  # 世界坐标系下的累积位置
        self.current_yaw = 0.0  # 当前航向角
        
        rospy.Subscriber("/control_data", controlData, self.control_cb)
        rospy.Timer(rospy.Duration(0.1), self.publish_tf)

    def control_cb(self, msg):
        """处理控制指令"""
        # 1. 先旋转
        self.current_yaw = np.deg2rad(msg.target_yaw / 100.0)  # 转换为弧度
        
        # 2. 后平移
        distance = msg.distance / 1000.0  # 假设distance单位为毫米转米
        self.robot_state = msg.robot_state

        if distance > 0:
            # 计算旋转后的位移分量
            dx = distance * np.cos(self.current_yaw)
            dy = distance * np.sin(self.current_yaw)
            
            # 更新累积位置
            self.position[0] = dx
            self.position[1] = dy

    def publish_tf(self, event):
        """发布TF变换"""
        transform = TransformStamped()
        
        # 头信息
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "odom"
        
        # 平移量（已包含旋转后的位移）
        transform.transform.translation.x = self.position[0]
        transform.transform.translation.y = self.position[1]
        transform.transform.translation.z = 0.0
        
        # 旋转量（绕Z轴）
        q = tf.transformations.quaternion_from_euler(0, 0, self.current_yaw)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    rospy.init_node('virtual_actuator')
    VirtualActuator()
    rospy.spin()