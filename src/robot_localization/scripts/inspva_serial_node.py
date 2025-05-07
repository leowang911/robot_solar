#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
from robot_localization.msg import INSPVA
from std_msgs.msg import Header

def compute_crc32(data_str):
    """修正顺序后的CRC-32/ISO-HDLC计算"""
    crc = 0xFFFFFFFF
    for byte in data_str.encode('ascii'):
        reversed_byte = int("{0:08b}".format(byte)[::-1], 2)
        crc ^= (reversed_byte << 24)
        for _ in range(8):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF
    crc = int("{0:032b}".format(crc)[::-1], 2)
    crc ^= 0xFFFFFFFF
    return crc

def parse_inspvae(line):
    """解析并验证字段及CRC"""
    if not line.startswith('$INSPVA'):
        return None
    
    if '*' not in line:
        return None
    data_part, checksum_part = line.split('*', 1)
    received_crc = checksum_part.strip()[:8].upper()
    
    crc_data = data_part[1:]
    computed_crc = compute_crc32(crc_data)
    computed_crc_str = "{:08X}".format(computed_crc)
    
    if computed_crc_str != received_crc:
        rospy.logwarn("CRC校验失败：计算值=%s，接收值=%s", computed_crc_str, received_crc)
        return None
    
    parts = data_part.split(',')
    if len(parts) != 30:
        rospy.logwarn("字段数量错误：期望30，实际=%d", len(parts))
        return None
    
    try:
        return {
            'week': int(parts[1]),
            'seconds': float(parts[2]),
            'latitude': float(parts[3]),
            'longitude': float(parts[4]),
            'altitude': float(parts[5]),
            've': float(parts[6]),
            'vn': float(parts[7]),
            'vu': float(parts[8]),
            'pitch': float(parts[9]),
            'roll': float(parts[10]),
            'yaw': float(parts[11]),
            'vc': float(parts[12]),
            'gyro_x': float(parts[13]),
            'gyro_y': float(parts[14]),
            'gyro_z': float(parts[15]),
            'acc_x': float(parts[16]),
            'acc_y': float(parts[17]),
            'acc_z': float(parts[18]),
            'quat_w': float(parts[19]),
            'quat_x': float(parts[20]),
            'quat_y': float(parts[21]),
            'quat_z': float(parts[22]),
            'NSV1': int(parts[23]),
            'NSV2': int(parts[24]),
            'age': int(parts[25]),
            'align_st': int(parts[26]),
            'nav_st': int(parts[27]),
            'odo_st': int(parts[28]),
            'reserve': parts[29]
        }
    except (ValueError, IndexError) as e:
        rospy.logerr("解析错误：%s", str(e))
        return None

def inspvae_serial_node():
    rospy.init_node('inspva_serial_node')
    
    port = rospy.get_param('~port', '/dev/rtkSerial')
    baudrate = rospy.get_param('~baudrate', 460800)
    topic_name = rospy.get_param('~topic', 'inspva_data')
    
    pub = rospy.Publisher(topic_name, INSPVA, queue_size=10)
    ser = None
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        rospy.loginfo("成功连接到串口: %s @ %d 波特率", port, baudrate)
        
        while not rospy.is_shutdown():
            raw_line = ser.readline()
            line = raw_line.decode('ascii', errors='ignore').strip()
            data = parse_inspvae(line)
            if data:
                msg = INSPVA()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'inspva'
                for field in data:
                    if hasattr(msg, field):
                        setattr(msg, field, data[field])
                pub.publish(msg)
            
    except serial.SerialException as e:
        rospy.logerr("串口错误: %s", str(e))
    except rospy.ROSInterruptException:
        pass
    finally:
        if ser and ser.is_open:
            ser.close()

if __name__ == '__main__':
    inspvae_serial_node()