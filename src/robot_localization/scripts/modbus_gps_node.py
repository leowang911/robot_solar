#!/usr/bin/env python
import rospy
import serial
import struct
import crcmod
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import QuaternionStamped
from robot_localization.msg import GPSData
from std_msgs.msg import Header
import math

class ModbusGPSNode:
    def __init__(self):
        rospy.init_node('modbus_gps_node')
        
        # 参数配置
        self.port = rospy.get_param('~port', '/dev/ttyUSB1')
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.slave_id = rospy.get_param('~slave_id', 0x50)
        self.polling_rate = rospy.get_param('~polling_rate', 100.0)
        
        # CRC16计算器
        self.crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF)
        
        # 发布器
        self.fix_pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
        self.yaw_pub = rospy.Publisher('gps/yaw', QuaternionStamped, queue_size=10)
        self.raw_pub = rospy.Publisher('gps/raw', GPSData, queue_size=10)
        
        # 初始化消息
        self.navsat_fix = NavSatFix()
        self.navsat_fix.header.frame_id = "gps"
        self.navsat_fix.status.service = NavSatStatus.SERVICE_GPS
        self.navsat_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        
        self.yaw_msg = QuaternionStamped()
        self.yaw_msg.header.frame_id = "gps"
        
        # 串口连接
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1.0
            )
            rospy.loginfo(f"Connected to {self.port} at {self.baudrate} baud")
        except Exception as e:
            rospy.logerr(f"Serial port error: {e}")
            rospy.signal_shutdown("Serial port error")
            return
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0/self.polling_rate), self.poll_data)

    def create_modbus_frame(self, reg_addr, num_regs):
        """创建MODBUS请求帧"""
        return struct.pack('>B B H H', 
                         self.slave_id, 
                         0x03, 
                         reg_addr, 
                         num_regs) + \
               struct.pack('<H', self.crc16(struct.pack('>B B H H', 
                                                      self.slave_id, 
                                                      0x03, 
                                                      reg_addr, 
                                                      num_regs)))

    def read_register(self, reg_addr):
        """读取单个寄存器"""
        try:
            self.ser.write(self.create_modbus_frame(reg_addr,1))
            response = self.ser.read(7)  # 响应长度固定7字节
            if len(response) != 7:
                return None
                
            # CRC校验
            crc_received = struct.unpack('<H', response[-2:])[0]
            if self.crc16(response[:-2]) != crc_received:
                return None
                
            # 解析数据 (返回值为无符号short)
            return struct.unpack('>H', response[3:5])[0]
        except Exception as e:
            rospy.logwarn(f"Read register error: {e}")
            return None

    def poll_data(self, event):
        try:
            # 1. 读取位置数据 (寄存器 0x0049-0x004C)
            pos_frame = self.create_modbus_frame(0x0049,4)
            self.ser.write(pos_frame)
            pos_response = self.ser.read(13)
            
            if not pos_response:
                rospy.logwarn("No position response")
                return
                
            pos_data = self.parse_position_response(pos_response)
            if not pos_data:
                return
            
            # 2. 读取高度 (寄存器 0x004D)
            altitude = self.read_register(0x004D)
            if altitude is None:
                rospy.logwarn("Altitude read failed")
                return
                
            # 3. 读取航向角 (寄存器 0x004E)
            yaw_raw = self.read_register(0x004E)
            if yaw_raw is None:
                rospy.logwarn("Yaw read failed")
                return
            
            # 转换为实际值
            yaw_deg = yaw_raw / 100.0  # 协议规定除以100
            
            # 发布NavSatFix
            self.publish_navsat_fix(pos_data['latitude'], 
                                  pos_data['longitude'], 
                                  altitude)
            
            # 发布航向角 (转换为四元数)
            self.publish_yaw(yaw_deg)
            
            # 发布原始数据
            self.publish_raw_data(pos_data['latitude'],
                                 pos_data['longitude'],
                                 altitude,
                                 yaw_deg)
            
        except Exception as e:
            rospy.logerr(f"Polling error: {e}")

    def parse_position_response(self, response):
        """解析位置响应数据"""
        if len(response) < 9:
            return None
            
        # CRC校验
        crc_received = struct.unpack('<H', response[-2:])[0]
        if self.crc16(response[:-2]) != crc_received:
            return None
            
        data = response[3:-2]
        lon_l, lon_h = struct.unpack('>2H', data[0:4])
        lat_l, lat_h = struct.unpack('>2H', data[4:8])
        
        lon = (lon_h << 16) | lon_l
        lat = (lat_h << 16) | lat_l
        
        return {
            'latitude': self._convert_to_decimal(lat),
            'longitude': self._convert_to_decimal(lon)
        }

    def _convert_to_decimal(self, value):
        """将原始值转换为十进制经纬度"""
        degrees = value // 10000000
        minutes = (value % 10000000) / 100000.0
        return degrees + minutes / 60.0

    def publish_navsat_fix(self, lat, lon, alt):
        """发布NavSatFix消息"""
        self.navsat_fix.header.stamp = rospy.Time.now()
        self.navsat_fix.latitude = lat
        self.navsat_fix.longitude = lon
        self.navsat_fix.altitude = float(alt)
        self.fix_pub.publish(self.navsat_fix)

    def publish_yaw(self, yaw_deg):
        """发布航向角(转换为四元数)"""
        yaw_rad = math.radians(yaw_deg)
        self.yaw_msg.header.stamp = rospy.Time.now()
        self.yaw_msg.quaternion.x = 0.0
        self.yaw_msg.quaternion.y = 0.0
        self.yaw_msg.quaternion.z = math.sin(yaw_rad/2)
        self.yaw_msg.quaternion.w = math.cos(yaw_rad/2)
        self.yaw_pub.publish(self.yaw_msg)

    def publish_raw_data(self, lat, lon, alt, yaw):
        """发布原始数据消息"""
        raw_msg = GPSData()
        raw_msg.header.stamp = rospy.Time.now()
        raw_msg.header.frame_id = "gps"
        raw_msg.latitude = lat
        raw_msg.longitude = lon
        raw_msg.altitude = float(alt)
        raw_msg.yaw = yaw
        self.raw_pub.publish(raw_msg)

if __name__ == '__main__':
    try:
        node = ModbusGPSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass