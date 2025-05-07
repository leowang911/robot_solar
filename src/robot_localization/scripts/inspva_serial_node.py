#!/usr/bin/env python
import rospy
import serial
from robot_localization.msg import INSPVA
from std_msgs.msg import Header
from __future__ import print_function 

def compute_crc32(data_str):
    """Compute CRC-32/ISO-HDLC checksum for given data string"""
    crc = 0xFFFFFFFF
    for byte in data_str.encode('ascii'):
        crc ^= byte << 24  # Reflect input byte
        for _ in range(8):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc <<= 1
        crc &= 0xFFFFFFFF  # Ensure CRC remains 32-bit
    
    # Reflect output and XOR with 0xFFFFFFFF
    crc = crc ^ 0xFFFFFFFF
    return crc

def parse_inspvae(line):
    """Parse $INSPVA message with CRC validation"""
    if not line.startswith('$INSPVA'):
        return None
    
    # Split checksum part
    if '*' not in line:
        rospy.logwarn("Invalid message: missing checksum delimiter")
        return None
    data_part, checksum_part = line.split('*', 1)
    
    # Prepare CRC calculation data (exclude $)
    crc_data = data_part[1:]
    
    # Compute and validate checksum
    computed_crc = compute_crc32(crc_data)
    received_crc = checksum_part.strip().split()[0][:8].upper()  # Handle trailing CR/LF
    computed_crc_str = "{:08X}".format(computed_crc)

    if computed_crc_str != received_crc:
        rospy.logwarn(f"Checksum mismatch: expected {computed_crc_str}, got {received_crc}")
        return None
    
    # Split data fields
    parts = data_part.split(',')
    if len(parts) < 26:
        rospy.logwarn(f"Invalid field count: Expected 26, got {len(parts)}")
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
            'NSV1': int(parts[19]),
            'NSV2': int(parts[20]),
            'age': int(parts[21]),
            'align_st': int(parts[22]),
            'nav_st': int(parts[23]),
            'odo_st': int(parts[24])
        }
    except (ValueError, IndexError) as e:
        rospy.logerr(f"Parsing error: {str(e)}")
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
        rospy.loginfo(f"Connected to {port} @ {baudrate} baud")
        
        while not rospy.is_shutdown():
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue
            
            if rospy.is_shutdown():
                break
                
            data = parse_inspvae(line)
            if not data:
                continue
            
            msg = INSPVAE()
            msg.header = Header(
                stamp=rospy.Time.now(),
                frame_id='inspva'
            )
            
            # Map parsed data to message fields
            for field in data:
                setattr(msg, field, data[field])
            
            pub.publish(msg)
            
    except serial.SerialException as e:
        rospy.logerr(f"Serial error: {str(e)}")
    except rospy.ROSInterruptException:
        pass
    finally:
        if ser and ser.is_open:
            ser.close()
            rospy.loginfo("Serial port closed")

if __name__ == '__main__':
    try:
        inspvae_serial_node()
    except rospy.ROSInterruptException:
        pass