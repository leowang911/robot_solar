#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion
class DroneDockingController:
    def __init__(self):
        rospy.init_node('drone_docking_controller')

        # 坐标系参数
        self.utm_zone = "50T"  # 根据实际位置设置UTM区
        self.robot_frame = "base_link"
        self.camera_frame = "orbbec_color_frame"

        # 控制参数
        self.gps_distance_threshold = 5.0  # 切换为视觉对准的GPS距离阈值（米）
        self.vision_distance_threshold = 0.1  # 最终对接精度（米）
        self.max_linear_speed = 0.5  # 最大线速度（m/s）
        self.max_angular_speed = 0.8  # 最大角速度（rad/s）
        self.kp_linear = 0.5  # 线性比例系数
        self.kp_angular = 1.2  # 角度比例系数

        # 状态变量
        self.robot_gps = None
        self.drone_gps = None
        self.drone_vision_pose = None  # 视觉检测的无人机位置（相机坐标系）
        self.state = "INIT"  # 状态机: INIT | GPS_NAV | VISION_NAV | DOCKED

        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅话题
        rospy.Subscriber("/robot/gps", NavSatFix, self.robot_gps_cb)
        rospy.Subscriber("/drone/gps", NavSatFix, self.drone_gps_cb)
        rospy.Subscriber("/aruco_single/pose", PoseStamped, self.aruco_pose_cb)

        # 发布控制指令
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def robot_gps_cb(self, msg):
        """机器人GPS回调：转换为UTM坐标"""
        self.robot_gps = self.gps_to_utm(msg.latitude, msg.longitude)

    def drone_gps_cb(self, msg):
        """无人机GPS回调：转换为UTM坐标"""
        self.drone_gps = self.gps_to_utm(msg.latitude, msg.longitude)

    def aruco_pose_cb(self, msg):
        """视觉ArUco位姿回调"""
        try:
            # 将位姿从相机坐标系转换到机器人基坐标系
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                msg.header.frame_id,
                rospy.Time(0)
            )
            pose_in_robot = self.transform_pose(msg.pose, transform)
            self.drone_vision_pose = pose_in_robot
        except tf2_ros.TransformException as e:
            rospy.logwarn("视觉坐标转换失败: %s", str(e))

    def gps_to_utm(self, lat, lon):
        """GPS经纬度转UTM坐标（需安装pyproj）"""
        from pyproj import Proj
        p = Proj(proj='utm', zone=self.utm_zone, ellps='WGS84')
        x, y = p(lon, lat)
        return (x, y)  # 返回平面坐标（米）

    def transform_pose(self, pose, transform):
        """坐标变换辅助函数"""
        # 此处应实现具体的坐标变换计算（略）
        return transformed_pose

    def calculate_distance(self, p1, p2):
        """计算两点间欧氏距离"""
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def gps_navigation(self):
        """GPS粗定位控制"""
        if self.robot_gps is None or self.drone_gps is None:
            return

        dx = self.drone_gps[0] - self.robot_gps[0]
        dy = self.drone_gps[1] - self.robot_gps[1]
        distance = self.calculate_distance(self.robot_gps, self.drone_gps)

        # 计算目标角度
        target_angle = math.atan2(dy, dx)
        current_angle = self.get_robot_yaw()  # 需实现获取机器人当前偏航角

        # 生成控制指令
        twist = Twist()
        twist.angular.z = self.kp_angular * (target_angle - current_angle)
        twist.linear.x = self.kp_linear * distance if distance > 0.5 else 0

        self.cmd_vel_pub.publish(twist)

        # 判断是否切换为视觉对准
        if distance <= self.gps_distance_threshold:
            self.state = "VISION_NAV"

    def vision_navigation(self):
        """视觉精对准控制"""
        if self.drone_vision_pose is None:
            return

        # 获取无人机在机器人坐标系中的位置
        x = self.drone_vision_pose.position.x
        y = self.drone_vision_pose.position.y
        distance = math.sqrt(x**2 + y**2)

        # 控制逻辑：对准并接近
        twist = Twist()
        if distance > self.vision_distance_threshold:
            # 对准方向
            target_angle = math.atan2(y, x)
            twist.angular.z = self.kp_angular * target_angle
            # 接近速度
            twist.linear.x = self.kp_linear * distance
        else:
            twist.linear.x = 0
            twist.angular.z = 0
            self.state = "DOCKED"

        self.cmd_vel_pub.publish(twist)

    def get_robot_yaw(self):
        """获取机器人当前偏航角（需订阅IMU或TF）"""
        # 实现代码略
        return 0.0

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == "INIT":
                if self.robot_gps and self.drone_gps:
                    self.state = "GPS_NAV"
            elif self.state == "GPS_NAV":
                self.gps_navigation()
            elif self.state == "VISION_NAV":
                self.vision_navigation()
            elif self.state == "DOCKED":
                rospy.loginfo("对接完成！")
                break
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = DroneDockingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass