#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
from io import BytesIO
from robot_localization.msg import INSPVA

class EnhancedDataBuffer:
    """增强型带容错的数据缓冲区"""
    def __init__(self):
        self.buffer = bytearray()
        self.start_marker = b'$INSPVA'
        self.min_frame_length = 30  # 最小有效帧长度
        self.max_frame_length = 1024
    
    def feed(self, data):
        self.buffer.extend(data)
        if len(self.buffer) > self.max_frame_length * 2:
            self._purge()

    def _purge(self):
        """清理无效前置数据"""
        start_pos = self.buffer.find(self.start_marker)
        if start_pos == -1:
            self.buffer.clear()
        else:
            del self.buffer[:start_pos]

    def extract_frame(self):
        while True:
            start = self.buffer.find(self.start_marker)
            if start == -1:
                return None
                
            end = self.buffer.find(b'\r\n', start)
            if end == -1:
                return None
                
            frame = self.buffer[start:end+2]
            del self.buffer[:end+2]
            
            if self.min_frame_length <= len(frame) <= self.max_frame_length:
                return frame.decode('ascii', errors='replace')
            
            rospy.logdebug(f"丢弃异常长度帧：{len(frame)} bytes")

def parse_inspvae_enhanced(line):
    """增强字段解析的健壮性"""
    line = line.strip()
    if not line.startswith('$INSPVA'):
        return None

    # 分离校验部分
    if '*' not in line:
        return None
    data_part, checksum_part = line.split('*', 1)
    received_checksum = checksum_part[:2].upper()
    
    # NMEA标准校验计算
    check_data = data_part[1:]  # 去掉$符号
    computed_checksum = 0
    for c in check_data:
        computed_checksum ^= ord(c)
    computed_checksum = f"{computed_checksum:02X}"
    
    if computed_checksum != received_checksum:
        rospy.logwarn(f"校验失败：计算值={computed_checksum} 接收值={received_checksum}")
        return None

    # 动态字段解析
    parts = data_part.split(',')
    field_map = {
        # 必要字段
        'week': (1, int),
        'seconds': (2, float),
        'latitude': (3, float),
        'longitude': (4, float),
        'altitude': (5, float),
        
        # 可选字段（带容错）
        've': (6, float),
        'vn': (7, float),
        'vu': (8, float),
        'pitch': (9, float),
        'roll': (10, float),
        'yaw': (11, float),
        'vc': (12, float),
        'gyro_x': (13, float),
        'gyro_y': (14, float),
        'gyro_z': (15, float),
        'acc_x': (16, float),
        'acc_y': (17, float),
        'acc_z': (18, float),
        'quat_w': (19, float),
        'quat_x': (20, float),
        'quat_y': (21, float),
        'quat_z': (22, float),
        'NSV1': (23, int),
        'NSV2': (24, int),
        'age': (25, int),
        'align_st': (26, int),
        'nav_st': (27, int),
        'odo_st': (28, int)
    }
    
    data = {}
    for field, (index, dtype) in field_map.items():
        if len(parts) > index:
            try:
                raw = parts[index].strip()
                data[field] = dtype(raw) if raw else 0
            except (ValueError, IndexError):
                data[field] = 0
                rospy.logdebug(f"字段解析失败：{field}[{index}] = '{parts[index]}'")
        else:
            data[field] = 0
            
    return data

def inspvae_serial_node():
    rospy.init_node('inspva_enhanced_node')
    
    port = rospy.get_param('~serial_port', '/dev/rtkSerial')
    baudrate = rospy.get_param('~baudrate', 460800)
    
    pub = rospy.Publisher('inspva', INSPVA, queue_size=10)
    buffer = EnhancedDataBuffer()
    
    try:
        with serial.Serial(port, baudrate, timeout=0.1) as ser:
            rospy.loginfo(f"成功打开串口：{port}@{baudrate}")
            
            while not rospy.is_shutdown():
                if ser.in_waiting > 0:
                    buffer.feed(ser.read(ser.in_waiting))
                    
                    while True:
                        frame = buffer.extract_frame()
                        if not frame:
                            break
                            
                        data = parse_inspvae_enhanced(frame)
                        if data:
                            msg = INSPVA()
                            msg.header.stamp = rospy.Time.now()
                            msg.header.frame_id = 'gnss'
                            
                            # 动态映射字段
                            for field in [
                                'week', 'seconds', 'latitude', 'longitude', 'altitude',
                                've', 'vn', 'vu', 'pitch', 'roll', 'yaw', 'vc',
                                'gyro_x', 'gyro_y', 'gyro_z', 'acc_x', 'acc_y', 'acc_z',
                                'quat_w', 'quat_x', 'quat_y', 'quat_z',
                                'NSV1', 'NSV2', 'age', 'align_st', 'nav_st', 'odo_st'
                            ]:
                                if field in data:
                                    setattr(msg, field, data[field])
                                    
                            pub.publish(msg)
                            
    except serial.SerialException as e:
        rospy.logerr(f"串口连接失败：{str(e)}")
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    inspvae_serial_node()