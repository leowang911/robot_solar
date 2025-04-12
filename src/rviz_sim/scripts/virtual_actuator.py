#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from robot_control.msg import controlData
import tf.transformations
import numpy as np

class VirtualActuator:
    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.current_pose = [0, 0, 0]  # x, y, yaw
        
        rospy.Subscriber("/control_data", controlData, self.control_cb)
        rospy.Timer(rospy.Duration(0.1), self.publish_tf)

    def control_cb(self, msg):
        """处理控制指令"""
        # 航向调整
        # if msg.target_yaw >= 18000:
        #     msg.target_yaw -= 36000
        # elif msg.target_yaw < -18000:
        #     msg.target_yaw += 36000

        target_yaw = msg.target_yaw / 100.0  # 0.01度转普通度
        self.current_pose[2] = np.deg2rad(target_yaw)
        
        # 直线运动
        if msg.distance > 0:
            dx = msg.distance / 1000.0 * np.sin(self.current_pose[2])
            dy = msg.distance / 1000.0 * np.cos(self.current_pose[2])
            self.current_pose[0] = -dx
            self.current_pose[1] = -dy
            self.distance = msg.distance / 1000.0



    def publish_tf(self, event):
        """发布机器人位姿"""
        
        self.rotation_euler = [0, 0, self.current_pose[2]]
        self.translation = [self.current_pose[0], self.current_pose[1], 0]
        # self.translation = [0,self.distance,0]

        # 生成旋转矩阵
        R = tf.transformations.euler_matrix(*self.rotation_euler)
        
        # 在旋转后的坐标系下进行平移
        rotated_translation = R[:3, :3] @ self.translation
        
        # 构造变换消息
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "odom"  # 父坐标系
        transform.child_frame_id = "base_link"    # 子坐标系
        
        # 设置平移量（在父坐标系中的坐标）
        transform.transform.translation.x = rotated_translation[0]
        transform.transform.translation.y = rotated_translation[1]
        transform.transform.translation.z = rotated_translation[2]
        
        # 设置旋转量（四元数）
        q = tf.transformations.quaternion_from_euler(0, 0, self.current_pose[2])
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]



        # self.tf_broadcaster.sendTransform(t)
        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    rospy.init_node('virtual_actuator')
    VirtualActuator()
    rospy.spin()