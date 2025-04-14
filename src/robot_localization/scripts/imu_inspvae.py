#!/usr/bin/env python
import rospy
import serial
import struct
from sensor_msgs.msg import Imu
from robot_localization.msg import INSPVAE
import numpy as np
from std_msgs.msg import Header

class IMUParser:
    def __init__(self):
        rospy.init_node('imu_parser_node')
        
        # 参数配置
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 230400)
        self.device_addr = 0x50  # 设备地址 (示例中的50)
        
        # 初始化串口
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        
        # 发布IMU数据
        self.imu_pub = rospy.Publisher('/inspvae_data', INSPVAE, queue_size=10)
        
        # 定时发送查询指令
        self.timer = rospy.Timer(rospy.Duration(0.02), self.send_query_cmd)

    def send_query_cmd(self, event):
        """发送查询指令: 50 03 00 3D 00 06 59 85"""
        cmd = bytes.fromhex(f"{self.device_addr:02X} 03 00 3D 00 06")
        crc = self.calculate_crc(cmd)
        full_cmd = cmd + crc
        rospy.loginfo(f"Sending command: {full_cmd.hex()}")
        self.ser.write(full_cmd)

    def parse_response(self, data):
        """解析返回数据"""
        if len(data) < 17 & data[1]!=50:  # 至少16字节: 地址(1)+功能码(1)+长度(1)+数据(12)+CRC(2)
            rospy.logwarn("数据长度不足")
            return None
        rospy.loginfo(f"Received data: {data.hex()}")
        # 校验CRC
        recv_crc = data[-2:]
        calc_crc = self.calculate_crc(data[:-2])
        if recv_crc != calc_crc:
            rospy.logwarn("CRC校验失败")
            return None
        
        # 提取角度数据
        l_roll_h, l_roll_l, h_roll_h, h_roll_l, \
        l_pitch_h, l_pitch_l, h_pitch_h, h_pitch_l, \
        l_yaw_h, l_yaw_l, h_yaw_h, h_yaw_l = struct.unpack('12B', data[3:15])
        
        # 计算角度 (小端模式)
        roll = np.int32(((h_roll_h << 24) | (h_roll_l << 16) | (l_roll_h << 8) | l_roll_l)) / 1000.0
        pitch = np.int32(((h_pitch_h << 24) | (h_pitch_l << 16) | (l_pitch_h << 8) | l_pitch_l)) / 1000.0
        yaw = np.int32(((h_yaw_h << 24) | (h_yaw_l << 16) | (l_yaw_h << 8) | l_yaw_l)) / 1000.0
        
        rospy.loginfo(f"Parsed angles: Roll={roll}, Pitch={pitch}, Yaw={yaw}")
        return roll, pitch, yaw

    def run(self):
        """主循环读取数据"""
        while not rospy.is_shutdown():
            if self.ser.in_waiting >= 17:
                data = self.ser.read(17)
                angles = self.parse_response(data)
                if angles:
                    self.publish_inspvae_data(*angles)

    def publish_inspvae_data(self, roll, pitch, yaw):
        """发布INSPVAE数据"""
        msg = INSPVAE()

        msg = INSPVAE()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'inspvae'
        
        # Assign all fields
        msg.week = 0
        msg.seconds = 0
        msg.latitude = 0
        msg.longitude = 0
        msg.altitude = 0
        msg.undulation = 0
        msg.std_lat = 0
        msg.std_lon = 0
        msg.std_alt = 0
        msg.ve = 0
        msg.vn = 0
        msg.vu = 0
        msg.std_ve = 0
        msg.std_vn = 0
        msg.std_vu = 0
        msg.pitch = 0
        msg.roll = 0
        msg.yaw = 0
        msg.std_pitch = 0
        msg.std_roll = 0
        msg.std_yaw = 0
        msg.ns = 0
        msg.gnss_st = 0
        msg.nav_st = 0
        msg.odo_st = 0
        msg.nav_status = '0'

        # if yaw & 0x80000000:  # 检查最高位是否为1
        #     yaw -= 0x100000000  # 转换为负数
        rospy.loginfo(f"Parsed angles: Roll={roll}, Pitch={pitch}, Yaw={yaw}")
        
        if yaw < 0:
            yaw += 360

        msg.roll = roll
        msg.pitch = pitch
        msg.yaw = yaw
        self.imu_pub.publish(msg)
        rospy.loginfo(f"Published IMU data: Roll={roll}, Pitch={pitch}, Yaw={yaw}")


    def publish_imu(self, roll, pitch, yaw):
        """发布IMU消息"""
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"
        
        # 转换为四元数 (示例，需根据实际坐标系调整)
        # 此处假设Roll-Pitch-Yaw为XYZ顺序
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        
        self.imu_pub.publish(msg)

    @staticmethod
    def calculate_crc(data):
        """Modbus CRC16校验 (示例中的CRC值为5985)"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

if __name__ == '__main__':
    node = IMUParser()
    node.run()