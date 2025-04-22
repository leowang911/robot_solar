#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, String, Float32, Int8
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from robot_control.msg import controlData  # 根据实际包名调整
from robot_localization.msg import baseStatus, INSPVAE, GPSData  # 根据实际包名调整


class TestDataPublisher:
    def __init__(self):
        rospy.init_node('test_data_publisher', anonymous=True)
        self.robot_state = rospy.get_param('~robot_state', '0')

        self.yaw = 0

        # 创建所有发布者
        # rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        # rospy.Subscriber('robot_state', UInt8, self.state_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.inspvae_callback)
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        # self.task_pub = rospy.Publisher('/task/start', Bool, queue_size=10)
        self.route_pub = rospy.Publisher('/mission/route_id', String, queue_size=10)
        self.battery_pub = rospy.Publisher('/battery/voltage', Float32, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_state_pub = rospy.Publisher('/robot_state', Int8, queue_size=10)
        self.control_data_pub = rospy.Publisher('/control_data', controlData, queue_size=10)
        self.inspvae_pub = rospy.Publisher('/inspvae', INSPVAE, queue_size=10)
        # 初始化默认消息
        self.init_messages()
        rospy.loginfo("Published zero test data")

    def inspvae_callback(self, msg):
        self.yaw = msg.yaw*100

    def init_messages(self):
        # 初始化IMU消息
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "base_link"
        # 加速度（单位：m/s²）
        self.imu_msg.linear_acceleration = Vector3(0.0, 0.0, 0.0)
        # 角速度（单位：rad/s）
        self.imu_msg.angular_velocity = Vector3(0.0, 0.0, 0.0)

        # 初始化GPS消息
        self.gps_msg = NavSatFix()
        self.gps_msg.header.frame_id = "gps"
        self.gps_msg.latitude = 40.123456  # 纬度
        self.gps_msg.longitude = 74.123456  # 经度
        self.gps_msg.status.status = -1  # 无效数据标志

        # 初始化任务状态
        self.task_msg = Bool()
        self.task_msg.data = False  # 任务未开始

        # 初始化航线编号
        self.route_msg = String()
        self.route_msg.data = "0"  # 默认航线编号

        # 初始化电池信息
        self.battery_msg = Float32()
        self.battery_msg.data = 0.0  # 电压0V

        # 初始化速度指令
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.linear.x = 1.0
        self.cmd_vel_msg.linear.y = 1.0
        self.cmd_vel_msg.linear.z = 1.0
        self.cmd_vel_msg.angular.x = 0.0
        self.cmd_vel_msg.angular.y = 0.0
        self.cmd_vel_msg.angular.z = 0.0

        # 初始化机器人状态
        self.robot_state_msg = Int8()
        self.robot_state_msg.data = 2  # 默认状态

        # 初始化控制数据
        self.control_data_msg = controlData()
        self.control_data_msg.header.frame_id = "control_data"
        self.control_data_msg.header.stamp = rospy.Time.now()
        self.control_data_msg.distance = 100 # 距离
        self.control_data_msg.roller_speed = 0
        self.control_data_msg.target_yaw = 0 # 目标航向角
        self.control_data_msg.yaw = int(self.yaw)   # 偏航角
        self.control_data_msg.robot_state = 3

            # MANUAL = 0, // 准备
            # IDLE,          // 空闲
            # AUTORUNING,    // 自动运行
            # UNLOADING,     // 出仓
            # LOADING,       // 入仓
            # ENERGENCUSTOP, // 急停
            # INIT,          // 重置
            # AUTOCORNER,    // 自动寻角
            # AUTOWASHING,   //  自动清洗    
    

        # 初始化INSPVAE消息
        self.inspvae_msg = INSPVAE()
        self.inspvae_msg.week = 0,
        self.inspvae_msg.seconds = 0.0,
        self.inspvae_msg.latitude = 0.0,
        self.inspvae_msg.longitude = 0.0,
        self.inspvae_msg.altitude = 0.0,
        self.inspvae_msg.undulation = 0.0,
        self.inspvae_msg.std_lat = 0.0,
        self.inspvae_msg.std_lon = 0.0,
        self.inspvae_msg.std_alt = 0.0,
        self.inspvae_msg.ve = 0.0,
        self.inspvae_msg.vn = 0.0,
        self.inspvae_msg.vu = 0.0,
        self.inspvae_msg.std_ve = 0.0,
        self.inspvae_msg.std_vn = 0.0,
        self.inspvae_msg.std_vu = 0.0,
        self.inspvae_msg.pitch = 0.0,
        self.inspvae_msg.roll = 0.0,
        self.inspvae_msg.yaw = 0.0,
        self.inspvae_msg.std_pitch = 0.0,
        self.inspvae_msg.std_roll = 0.0,
        self.inspvae_msg.std_yaw = 0.0,
        self.inspvae_msg.ns = 0,
        self.inspvae_msg.gnss_st = 0,
        self.inspvae_msg.nav_st = 0,
        self.inspvae_msg.odo_st = 0,
        self.inspvae_msg.nav_status = 0
        




    def publish_data(self):
        rate = rospy.Rate(100)  # 100Hz发布频率
        # # 发布初始消息
        # self.control_data_msg = controlData()
        # self.control_data_msg.header.frame_id = "control_data"
        # self.control_data_msg.header.stamp = rospy.Time.now()
        # self.control_data_msg.distance = 0 # 距离
        # self.control_data_msg.roller_speed = 0
        # self.control_data_msg.target_yaw = 0  # 目标航向角
        # self.control_data_msg.yaw = 0  # 偏航角
        # self.control_data_msg.robot_state = 1
        # self.control_data_pub.publish(self.control_data_msg)

        while not rospy.is_shutdown():
            # 更新时间戳
            current_time = rospy.Time.now()
            self.imu_msg.header.stamp = current_time
            self.gps_msg.header.stamp = current_time

            # 发布所有消息
            self.imu_pub.publish(self.imu_msg)
            self.gps_pub.publish(self.gps_msg)
            # self.task_pub.publish(self.task_msg)
            self.route_pub.publish(self.route_msg)
            self.battery_pub.publish(self.battery_msg)
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            self.robot_state_pub.publish(self.robot_state_msg)
            self.control_data_pub.publish(self.control_data_msg)
            self.inspvae_pub.publish(self.inspvae_msg)
            # 
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TestDataPublisher()
        publisher.publish_data()
    except rospy.ROSInterruptException:
        pass
