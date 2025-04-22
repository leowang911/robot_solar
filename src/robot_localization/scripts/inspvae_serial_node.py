#!/usr/bin/env python
import rospy
import serial
from robot_localization.msg import INSPVAE
from std_msgs.msg import Header

def parse_inspvae(line):
    """Parse $INSPVAE message and return dictionary of values"""
    if not line.startswith('$INSPVAE'):
        return None
    
    parts = line.split(',')
    if len(parts) < 27:
        rospy.logwarn("Invalid INSPVAE message: not enough fields")
        return None
    
    try:
        data = {
            'week': int(parts[1]),
            'seconds': float(parts[2]),
            'latitude': float(parts[3]),
            'longitude': float(parts[4]),
            'altitude': float(parts[5]),
            'undulation': float(parts[6]),
            'std_lat': float(parts[7]),
            'std_lon': float(parts[8]),
            'std_alt': float(parts[9]),
            've': float(parts[10]),
            'vn': float(parts[11]),
            'vu': float(parts[12]),
            'std_ve': float(parts[13]),
            'std_vn': float(parts[14]),
            'std_vu': float(parts[15]),
            'pitch': float(parts[16]),
            'roll': float(parts[17]),
            'yaw': float(parts[18]),
            'std_pitch': float(parts[19]),
            'std_roll': float(parts[20]),
            'std_yaw': float(parts[21]),
            'ns': int(parts[22]),
            'gnss_st': int(parts[23]),
            'nav_st': int(parts[24]),
            'odo_st': int(parts[25]),
            'nav_status': parts[26].split('*')[0]  # Remove checksum if present
        }
        return data
    except ValueError as e:
        rospy.logerr(f"Error parsing INSPVAE message: {e}")
        return None

# def inspvae_serial_node():
#     rospy.init_node('inspvae_serial_node')
    
#     port = rospy.get_param('~port', '/dev/ttyUSB1')
#     baudrate = rospy.get_param('~baudrate', 460800)
#     topic_name = rospy.get_param('~topic', 'inspvae_data')
    
#     pub = rospy.Publisher(topic_name, INSPVAE, queue_size=10)
#     try:
#         # 显式配置所有串口参数
#         ser = serial.Serial(
#             port=port,
#             baudrate=baudrate,
#             bytesize=serial.EIGHTBITS,
#             parity=serial.PARITY_NONE,
#             stopbits=serial.STOPBITS_ONE,
#             timeout=1,
#             rtscts=False,
#             dsrdtr=False
#         )
#         rospy.loginfo(f"成功打开串口: {ser.name}")
        
#         # 首次读取测试
#         test_data = ser.readline()
#         rospy.loginfo(f"首次测试数据: {test_data}")
        
#         while not rospy.is_shutdown():
#             if ser.in_waiting > 0:
#                 line = ser.readline().decode('ascii', errors='ignore').strip()
#                 data = parse_inspvae(line)
#                 if data:
#                     # 发布消息逻辑
#                     msg = INSPVAE()
#                     # ... 填充消息字段
#                     pub.publish(msg)
#             else:
#                 rospy.sleep(0.001)  # 避免忙等待
            
#     except serial.SerialException as e:
#         rospy.logerr(f"串口异常: {e}")
#         rospy.signal_shutdown("串口错误")
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         if 'ser' in locals() and ser.is_open:
#             ser.close()
#             rospy.loginfo("串口已关闭")



def inspvae_serial_node():
    rospy.init_node('inspvae_serial_node')
    
    # Get parameters
    port = rospy.get_param('~port', '/dev/ttyUSB1')
    baudrate = rospy.get_param('~baudrate', 460800)
    topic_name = rospy.get_param('~topic', 'inspvae_data')
    
    # Setup publisher
    pub = rospy.Publisher(topic_name, INSPVAE, queue_size=10)
    
    try:
        # Initialize serial connection
        ser = serial.Serial(port, baudrate, timeout=1)
        rospy.loginfo(f"Connected to {port} at {baudrate} baud")
        
        while not rospy.is_shutdown():
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue
                
            # Parse INSPVAE message7+9
            data = parse_inspvae(line)
            if not data:
                continue
                
            # Create and publish ROS message
            msg = INSPVAE()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'inspvae'
            
            # Assign all fields
            msg.week = data['week']
            msg.seconds = data['seconds']
            msg.latitude = data['latitude']
            msg.longitude = data['longitude']
            msg.altitude = data['altitude']
            msg.undulation = data['undulation']
            msg.std_lat = data['std_lat']
            msg.std_lon = data['std_lon']
            msg.std_alt = data['std_alt']
            msg.ve = data['ve']
            msg.vn = data['vn']
            msg.vu = data['vu']
            msg.std_ve = data['std_ve']
            msg.std_vn = data['std_vn']
            msg.std_vu = data['std_vu']
            msg.pitch = data['pitch']
            msg.roll = data['roll']
            msg.yaw = data['yaw']
            msg.std_pitch = data['std_pitch']
            msg.std_roll = data['std_roll']
            msg.std_yaw = data['std_yaw']
            msg.ns = data['ns']
            msg.gnss_st = data['gnss_st']
            msg.nav_st = data['nav_st']
            msg.odo_st = data['odo_st']
            msg.nav_status = data['nav_status']
            
            pub.publish(msg)
            
    except serial.SerialException as e:
        rospy.logerr(f"Serial port error: {e}")
    except rospy.ROSInterruptException:
        ser.close()
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    try:
        inspvae_serial_node()
    except rospy.ROSInterruptException:
        pass