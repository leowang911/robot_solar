#!/usr/bin/env python
import rospy
import serial
import struct
from robot_localization.msg import baseStatus
from std_msgs.msg import Header

class WheelDataParser:
    def __init__(self):
        rospy.init_node('wheel_data_parser')
        
        # 参数配置
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 115200)
        self.frame_length = 9  # 根据数据帧结构计算的总长度
        
        # 初始化发布器
        self.pub = rospy.Publisher('wheel_status', baseStatus, queue_size=10)
        
        # 配置串口
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            rospy.loginfo(f"Connected to {self.port} at {self.baud} baud")
        except serial.SerialException as e:
            rospy.logerr(f"Serial port error: {e}")
            rospy.signal_shutdown("Serial port error")
            return
        
        # 接收缓冲区
        self.buffer = bytearray()
        
    def parse_frame(self, data):
        """解析数据帧"""
        try:
            # 验证帧头
            if data[0] != 0xAA:
                rospy.logwarn("Invalid frame header")
                return None
            
            # 解析各字段（使用大端序）
            speed = struct.unpack('>h', bytes(data[1:3]))[0]
            angle = struct.unpack('>h', bytes(data[3:5]))[0]
            sensor_state = data[5]
            complete_state = data[7]
            rc_state = data[8]
            
            return {
                'speed': speed,
                'angle': angle,
                'sensor_state': sensor_state,
                'complete_state': complete_state,
                'rc_state': rc_state
            }
        except Exception as e:
            rospy.logerr(f"Parsing error: {e}")
            return None
    
    def process_buffer(self):
        """处理缓冲区数据"""
        while len(self.buffer) >= self.frame_length:
            # 查找帧头
            header_pos = self.buffer.find(b'\xAA')
            if header_pos == -1:
                self.buffer.clear()
                return
            
            # 检查是否有完整帧
            if len(self.buffer) - header_pos < self.frame_length:
                return
                
            # 提取完整帧
            frame = self.buffer[header_pos:header_pos+self.frame_length]
            self.buffer = self.buffer[header_pos+self.frame_length:]
            
            # 解析数据
            parsed = self.parse_frame(frame)
            if parsed:
                self.publish_data(parsed)
    
    def publish_data(self, data):
        """发布ROS消息"""
        msg = baseStatus()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        msg.speed = data['speed']
        msg.angle = data['angle']
        msg.sensor_state = data['sensor_state']
        msg.complete_state = data['complete_state']
        msg.rc_state = data['rc_state']
        
        self.pub.publish(msg)
    
    def run(self):
        rate = rospy.Rate(100)  # 100Hz
        while not rospy.is_shutdown():
            # 读取串口数据
            try:
                data = self.ser.read_all()
                if data:
                    self.buffer.extend(data)
                    self.process_buffer()
            except serial.SerialException as e:
                rospy.logerr(f"Serial read error: {e}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        parser = WheelDataParser()
        parser.run()
    except rospy.ROSInterruptException:
        pass