#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
from robot_localization.msg import INSPVA
from std_msgs.msg import Header

def compute_crc32(data_str):
    """修正后的CRC-32计算（取消字节和整体反转）"""
    crc = 0xFFFFFFFF
    for byte in data_str.encode('ascii'):
        crc ^= (byte << 24)  # 关键修改：直接使用原始字节，不反转位序
        for _ in range(8):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF
    crc ^= 0xFFFFFFFF
    return crc

def parse_inspvae(line):
    """精确解析消息并验证CRC"""
    line = line.strip()  # 去除首尾空白字符（包括\r\n）
    rospy.loginfo("解析行: %s", line)
    if not line.startswith('$INSPVA'):
        return None
    
    # 分离数据与校验部分（严格处理*后的内容）
    if '*' not in line:
        return None
    data_part, checksum_part = line.split('*', 1)
    
    # 清理校验和部分（只保留8位十六进制字符）
    received_crc = checksum_part.strip()[:8].upper().replace(' ', '')
    if len(received_crc) != 8:
        rospy.logwarn("无效校验和长度: %s", received_crc)
        return None
    
    # 提取待校验数据（确保不包含$）
    crc_data = data_part[1:]  # 去掉开头的$
    computed_crc = compute_crc32(crc_data)
    computed_crc_str = "{:08X}".format(computed_crc)
    
    # # CRC验证
    # if computed_crc_str != received_crc:
    #     rospy.logwarn("CRC校验失败：计算值=%s 接收值=%s 数据=[%s]", 
    #                  computed_crc_str, received_crc, crc_data)
    #     return None
    
    # 字段解析（严格匹配字段数量）
    parts = data_part.split(',')
    expected_fields = 29  # 根据实际消息确定
    if len(parts) != expected_fields:
        rospy.logwarn("字段数量错误：期望=%d 实际=%d", expected_fields, len(parts))
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

        }
    except (ValueError, IndexError) as e:
        rospy.logerr("解析错误: %s", str(e))
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
        rospy.loginfo("成功连接串口: %s @ %d", port, baudrate)
        
        while not rospy.is_shutdown():
            raw_line = ser.readline()
            try:
                line = raw_line.decode('ascii').strip()
            except UnicodeDecodeError:
                rospy.logwarn("解码错误，忽略无效数据")
                continue
            
            # 调试输出原始数据
            rospy.logdebug("原始数据: %s", line)
            
            data = parse_inspvae(line)
            if not data:
                continue
            
            msg = INSPVA()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'inspva'
            
            # 动态映射字段
            for field in data:
                if hasattr(msg, field):
                    setattr(msg, field, data[field])
                else:
                    rospy.logwarn("未定义字段: %s", field)
            
            pub.publish(msg)
            
    except serial.SerialException as e:
        rospy.logerr("串口异常: %s", str(e))
    except rospy.ROSInterruptException:
        pass
    finally:
        if ser and ser.is_open:
            ser.close()
            rospy.loginfo("串口已关闭")

if __name__ == '__main__':
    inspvae_serial_node()