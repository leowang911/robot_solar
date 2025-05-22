#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
from io import BytesIO
from robot_localization.msg import INSPVA
from std_msgs.msg import Header

class DataBuffer:
    """增强型数据缓冲区管理"""
    def __init__(self):
        self.buffer = BytesIO()
        self.start_marker = b'$INSPVA'
        self.end_marker = b'*'
        self.max_length = 1024  # 最大允许单帧长度
        
    def append(self, data):
        if len(self.buffer.getvalue()) + len(data) > self.max_length:
            self.buffer = BytesIO()  # 防止缓冲区溢出
        self.buffer.write(data)
        
    def extract_frame(self):
        """提取完整数据帧并清理缓冲区"""
        buffer_bytes = self.buffer.getvalue()
        start_idx = buffer_bytes.find(self.start_marker)
        if start_idx == -1:
            return None, 0
            
        end_idx = buffer_bytes.find(self.end_marker, start_idx)
        if end_idx == -1:
            return None, 0
            
        # 提取完整帧（包含校验部分）
        frame_end = buffer_bytes.find(b'\r\n', end_idx)
        if frame_end == -1:
            return None, 0
            
        full_frame = buffer_bytes[start_idx:frame_end+2]
        remaining = buffer_bytes[frame_end+2:]
        
        # 重置缓冲区
        self.buffer = BytesIO()
        self.buffer.write(remaining)
        return full_frame.decode('ascii', errors='ignore'), start_idx

def compute_checksum(data_str):
    """NMEA标准异或校验（根据实际情况选择）"""
    checksum = 0
    for c in data_str:
        checksum ^= ord(c)
    return "{:02X}".format(checksum)

def parse_inspvae(line):
    """增强型解析方法"""
    line = line.strip()
    if not line.startswith('$INSPVA'):
        return None
        
    # 分离校验部分
    if '*' not in line:
        return None
    data_part, checksum_part = line.split('*', 1)
    
    # 计算校验和（示例使用NMEA标准异或校验）
    check_data = data_part[1:]  # 去掉$符号
    computed_crc = compute_checksum(check_data)
    received_crc = checksum_part[:2].upper()
    
    # if computed_crc != received_crc:
    #     rospy.logwarn(f"Checksum mismatch: {computed_crc} vs {received_crc}")
    #     return None
        
    # 字段解析（动态适配字段数量）
    parts = data_part.split(',')
    try:
        base_data = {
            'week': int(parts[1]),
            'seconds': float(parts[2]),
            'latitude': float(parts[3]),
            'longitude': float(parts[4]),
            'altitude': float(parts[5]),
            've': float(parts[6]),
            'vn': float(parts[7]),
            'vu': float(parts[8])
        }
        
        # 动态处理可选字段
        optional_fields = {
            'pitch': 9, 'roll': 10, 'yaw': 11,
            'vc': 12, 'gyro_x': 13, 'gyro_y': 14, 'gyro_z': 15,
            'acc_x': 16, 'acc_y': 17, 'acc_z': 18
        }
        
        for field, index in optional_fields.items():
            if len(parts) > index:
                base_data[field] = float(parts[index])
                
        return base_data
    except (IndexError, ValueError) as e:
        rospy.logerr(f"Parse error: {str(e)} in line: {line}")
        return None

def inspvae_serial_node():
    rospy.init_node('inspva_serial_node')
    
    port = rospy.get_param('~port', '/dev/rtkSerial')
    baudrate = rospy.get_param('~baudrate', 460800)
    topic_name = rospy.get_param('~topic', 'inspva_data')
    
    pub = rospy.Publisher(topic_name, INSPVA, queue_size=10)
    buffer = DataBuffer()
    
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            rospy.loginfo(f"Connected to {port} @ {baudrate}")
            
            while not rospy.is_shutdown():
                raw_data = ser.read(ser.in_waiting or 1)
                if raw_data:
                    buffer.append(raw_data)
                    
                    while True:
                        frame, pos = buffer.extract_frame()
                        if not frame:
                            break
                            
                        data = parse_inspvae(frame)
                        if data:
                            msg = INSPVA()
                            msg.header.stamp = rospy.Time.now()
                            msg.header.frame_id = 'inspva'
                            
                            for field in ['week', 'seconds', 'latitude', 'longitude',
                                        'altitude', 've', 'vn', 'vu', 'pitch',
                                        'roll', 'yaw', 'vc', 'gyro_x', 'gyro_y',
                                        'gyro_z', 'acc_x', 'acc_y', 'acc_z']:
                                if field in data:
                                    setattr(msg, field, data[field])
                                    
                            pub.publish(msg)
                        else:
                            rospy.logdebug(f"Discarded invalid frame at pos {pos}")
                            
    except Exception as e:
        rospy.logerr(f"Serial error: {str(e)}")
    finally:
        pub.unregister()

if __name__ == '__main__':
    inspvae_serial_node()