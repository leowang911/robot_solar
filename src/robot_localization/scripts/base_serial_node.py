#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import math
from robot_localization.msg import baseStatus,INSPVAE  # 根据实际包名调整
from robot_control.msg import controlData  # 根据实际包名调整
import numpy as np

class BaseSerialNode:
    def __init__(self):
        rospy.init_node('base_serial_node')

        # 参数配置
        self.port = rospy.get_param('~port', '/dev/baseSerial')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.angle_dir = rospy.get_param('~angle_dir', 1)
        self.rx_frame_length = 14       # 接收帧长度
        self.tx_frame_length = 13       # 发送帧长度

        # 状态变量
        self.speed = 0
        self.distance = 0
        self.sensor_state = 0
        self.complete_state = 1
        self.complete_state_prev = 0
        self.current_yaw = 0
        self.rc_state = 0
        self.rc_state_prev = 0
        self.yaw_prev = 0
        self.distance_prev = 0
        self.robot_state_prev = 1
        self.voltage = 0
        self.last_tx_data = None
        self.last_tx_data_prev = {
            'distance': 0,
            'target_yaw': 0,
            'roller_speed': 0,
            'yaw':0,
            'robot_state': 1,
            'voltage': 0,
            'error': 0
        }

        # 初始化串口
        self.ser = None
        self.init_serial()

        # 发布接收数据
        self.wheel_pub = rospy.Publisher('base_status', baseStatus, queue_size=10)

        # 订阅控制指令
        rospy.Subscriber('/control_data', controlData, self.control_data_callback)
        rospy.Subscriber('/inspvae_data',INSPVAE,self.inspvae_cb)

    def inspvae_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.current_yaw = self.angle_dir*msg.yaw*100
    

    def init_serial(self):
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1  # 设置适当的超时时间
            )
            rospy.loginfo(f"Connected to {self.port} at {self.baudrate} baud")
        except serial.SerialException as e:
            rospy.logerr(f"Serial port error: {e}")
            rospy.signal_shutdown("Serial port init failed")

    def control_data_callback(self, msg):
        """速度指令回调"""
        self.last_tx_data = {
            'distance': msg.distance,
            'target_yaw': msg.target_yaw,
            'roller_speed': msg.roller_speed,
            'yaw': self.current_yaw,
            'robot_state': msg.robot_state
        }
        # rospy.loginfo(f'yaw: {self.current_yaw}')

    def parse_rx_frame(self, data):
        """解析接收数据帧"""
        try:
            if data[0] != 0xAA or len(data) != self.rx_frame_length or data[13] != sum(data[:13]) & 0xFF:
                rospy.logwarn("Invalid frame or checksum error")
                return None
            
            # 解析各字段（大端序）
            self.speed = struct.unpack('>h', data[1:3])[0]
            self.distance = struct.unpack('>i', data[3:7])[0]
            self.sensor_state = data[7]
            self.complete_state = data[9]     ####初始设置为1
            self.rc_state = data[10]
            self.voltage = data[11]
            self.error = data[12]
            return {
                'speed': self.speed,
                'distance': self.distance,
                'sensor_state': self.sensor_state,
                'complete_state': self.complete_state,
                'rc_state': self.rc_state,
                'voltage': self.voltage,
                'error': self.error
            }

        except Exception as e:
            rospy.logerr(f"Parse error: {e}")
            return None

    def create_tx_frame(self, data):
        """创建发送数据帧"""
        if data is None:
            return None

        state = data.get('robot_state', 0x00)

        if self.rc_state_prev != 2 and self.rc_state == 2:
            self.last_tx_data_prev = data
            state = 0x01

        if self.complete_state == 1 or state == 0x01:
            self.last_tx_data_prev = data

        # if state == 0x02 and self.complete_state_prev == 0 and self.complete_state ==1:
        #     self.last_tx_data_prev = data
        #     # rospy.loginfo("robot_state: 0x02, complete_state_prev: 0, complete_state: 1")
        #     state = 0x01
 
        tx_distance = int(self.last_tx_data_prev.get('distance', 0.0))
        tx_target_yaw = np.int16(self.last_tx_data_prev.get('target_yaw', 0.0))
        tx_roller_speed = np.uint16(self.last_tx_data_prev.get('roller_speed', 0.0))
        # tx_yaw = np.uint16(data.get('yaw', 0.0))
        tx_yaw = np.int16(self.current_yaw)

        rospy.loginfo(f"tx_distance: {tx_distance}, tx_target_yaw: {tx_target_yaw}, tx_roller_speed: {tx_roller_speed}, tx_yaw: {tx_yaw}, state: {state}")

        # if state == 0x02 and self.complete_state_prev ==0 and self.complete_state == 1:
        #     state = 0x01

        
 
        # if state == 0x02 & self.complete_state == 0:
        #     tx_distance = self.distance_prev
        #     tx_target_yaw = self.yaw_prev


        frame = struct.pack('<BiHHHB',
                            0x55,
                            tx_distance,    
                            tx_target_yaw & 0xFFFF,
                            tx_roller_speed & 0xFFFF,
                            tx_yaw & 0xFFFF,
                            state)

        # 计算校验和
        checksum = sum(frame[:12]) & 0xFF
        final_frame = frame + bytes([checksum])
        # rospy.loginfo(f"Creating frame: {final_frame.hex()}")
        self.complete_state_prev = self.complete_state
        self.rc_state_prev  = self.rc_state
        return final_frame

    def publish_wheel_status(self, data):
        """发布车轮状态"""
        msg = baseStatus()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.speed = data['speed']
        msg.distance = data['distance']
        msg.sensor_state = data['sensor_state']
        msg.complete_state = data['complete_state']
        msg.rc_state = data['rc_state']
        msg.voltage = data['voltage']
        msg.error = data['error']
        self.wheel_pub.publish(msg)

    def run(self):
        """主循环"""
        buffer = bytearray()
        while not rospy.is_shutdown():
            try:
                # 1. 尝试读取串口数据
                if self.ser.in_waiting > 0:
                    buffer += self.ser.read(self.ser.in_waiting)

                # 2. 处理接收到的完整帧
                if len(buffer) >= self.rx_frame_length:
                    # 查找帧头
                    header_pos = buffer.find(b'\xAA')
                    if header_pos >= 0 and len(buffer) >= header_pos + self.rx_frame_length:
                        # 提取完整帧
                        frame = buffer[header_pos:header_pos+self.rx_frame_length]
                        buffer = buffer[header_pos+self.rx_frame_length:]

                        # 解析数据
                        parsed = self.parse_rx_frame(frame)
                        rospy.loginfo(f"Received frame: {frame.hex()}")
                        if parsed:
                            self.publish_wheel_status(parsed)

                            # 3. 收到完整帧后立即发送数据
                            if self.last_tx_data is not None:
                                tx_frame = self.create_tx_frame(self.last_tx_data)
                                if tx_frame is not None and len(tx_frame) == self.tx_frame_length:
                                    self.ser.write(tx_frame)
                                    # rospy.logdebug(f"Sent frame after receiving: {tx_frame.hex()}")

                # 控制循环频率
                rospy.sleep(0.001)

            except serial.SerialException as e:
                rospy.logerr(f"Serial communication error: {e}")
                self.init_serial()  # 尝试重新初始化串口
                rospy.sleep(1)
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")
                rospy.sleep(1)

    def shutdown(self):
        """安全关闭"""
        if self.ser and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    node = BaseSerialNode()
    rospy.on_shutdown(node.shutdown)
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass