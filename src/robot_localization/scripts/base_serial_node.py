#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import math
from robot_localization.msg import baseStatus, INSPVAE
from robot_control.msg import controlData
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse
import time

class BaseSerialNode:
    def __init__(self):
        rospy.init_node('base_serial_node')

        # 参数配置
        self.port = rospy.get_param('~port', '/dev/baseSerial')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.angle_dir = rospy.get_param('~angle_dir', -1)
        self.rx_frame_length = 14       # 接收帧长度
        self.tx_frame_length = 13       # 发送帧长度
        self.max_reconnect_attempts = 10  # 最大重连尝试次数
        self.reconnect_delay = 1.0       # 重连延迟(秒)
        self.heartbeat_interval = 0.5    # 心跳间隔(秒)
        self.last_heartbeat_time = 0     # 上次心跳时间

        # 状态变量
        self.stop_flag = False
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
        self.last_valid_rx_time = 0  # 上次有效接收时间
        self.comm_timeout = 1.0      # 通信超时时间(秒)
        self.reconnect_attempts = 0  # 当前重连尝试次数

        # 初始化串口
        self.ser = None
        self.init_serial()

        # 发布接收数据
        self.wheel_pub = rospy.Publisher('base_status', baseStatus, queue_size=10)

        # 订阅控制指令
        rospy.Subscriber('/control_data', controlData, self.control_data_callback)
        rospy.Subscriber('/inspvae_data', INSPVAE, self.inspvae_cb)

        self.service_stop = rospy.Service(
            '/emergency_stop', Trigger, self.handle_emergency_stop
        )

        self.service_reboot = rospy.Service(
            '/reboot', Trigger, self.handle_reboot
        )

    def inspvae_cb(self, msg):
        self.current_yaw = math.radians(msg.yaw)

    def init_serial(self):
        """初始化串口连接，支持重试机制"""
        self.reconnect_attempts = 0
        while not rospy.is_shutdown() and self.reconnect_attempts < self.max_reconnect_attempts:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1
                )
                rospy.loginfo(f"Connected to {self.port} at {self.baudrate} baud")
                self.last_valid_rx_time = rospy.get_time()  # 重置有效接收时间
                return True
            except (serial.SerialException, OSError) as e:
                self.reconnect_attempts += 1
                rospy.logerr(f"Serial port error (attempt {self.reconnect_attempts}/{self.max_reconnect_attempts}): {e}")
                rospy.sleep(self.reconnect_delay)
        
        rospy.logerr("Failed to initialize serial port after multiple attempts")
        return False

    def control_data_callback(self, msg):
        """速度指令回调"""
        self.last_tx_data = {
            'distance': msg.distance,
            'target_yaw': msg.target_yaw,
            'roller_speed': msg.roller_speed,
            'yaw': self.yaw_to_target_yaw_angle(self.current_yaw, 0),
            'robot_state': msg.robot_state
        }

    def parse_rx_frame(self, data):
        """解析接收数据帧，增加冗余校验"""
        try:
            # 基本校验
            if len(data) < self.rx_frame_length:
                rospy.logwarn(f"Incomplete frame: expected {self.rx_frame_length} bytes, got {len(data)}")
                return None
            
            if data[0] != 0xAA:
                rospy.logwarn(f"Invalid frame header: 0x{data[0]:02X} (expected 0xAA)")
                return None
            
            # 计算校验和
            calculated_checksum = sum(data[:13]) & 0xFF
            received_checksum = data[13]
            
            if calculated_checksum != received_checksum:
                rospy.logwarn(f"Checksum error: calculated 0x{calculated_checksum:02X}, received 0x{received_checksum:02X}")
                return None
            
            # 解析各字段（大端序）
            speed = struct.unpack('>h', data[1:3])[0]
            distance = struct.unpack('>i', data[3:7])[0]
            sensor_state = data[7]
            complete_state = data[9]
            rc_state = data[10]
            voltage = data[11]
            error = data[12]
            
            # 更新状态变量
            self.speed = speed
            self.distance = distance
            self.sensor_state = sensor_state
            self.complete_state = complete_state
            self.rc_state = rc_state
            self.voltage = voltage
            self.error = error
            
            # 更新最后有效接收时间
            self.last_valid_rx_time = rospy.get_time()
            
            return {
                'speed': speed,
                'distance': distance,
                'sensor_state': sensor_state,
                'complete_state': complete_state,
                'rc_state': rc_state,
                'voltage': voltage,
                'error': error
            }

        except Exception as e:
            rospy.logerr(f"Parse error: {e}")
            return None

    def yaw_to_target_yaw_angle(self, yaw, current_yaw):
        """将航向角转换为控制角度"""
        angle = self.angle_dir * (math.degrees(yaw) * 100) + math.degrees(current_yaw) * 100
        angle = angle % 36000
        if angle < 0:
            angle += 36000
        return np.uint16(angle)

    def handle_emergency_stop(self, req):
        """紧急停止服务处理"""
        try:
            self.stop_flag = True
            rospy.loginfo("Executing emergency stop...")
            return TriggerResponse(success=True, message="Emergency stop executed successfully")
        except Exception as e:
            rospy.logerr(f"Emergency stop failed: {str(e)}")
            return TriggerResponse(success=False, message=f"Error during emergency stop: {str(e)}")

    def handle_reboot(self, req):
        """重启服务处理"""
        try:
            self.stop_flag = False
            rospy.loginfo("Rebooting communication...")
            self.init_serial()  # 尝试重新初始化串口
            return TriggerResponse(success=True, message="Reboot executed successfully")
        except Exception as e:
            rospy.logerr(f"Reboot failed: {str(e)}")
            return TriggerResponse(success=False, message=f"Error during reboot: {str(e)}")

    def create_tx_frame(self, data):
        """创建发送数据帧，增加安全检查"""
        if data is None:
            return None

        state = data.get('robot_state', 0x00)
        
        # 紧急停止状态处理
        if self.stop_flag:
            state = 0x01
            rospy.logwarn_throttle(1.0, "Emergency stop active, sending stop command")
        
        # 使用上一次的有效数据
        if self.complete_state != 0 or state == 0x01:
            self.last_tx_data_prev = data

        tx_distance = int(self.last_tx_data_prev.get('distance', 0.0))
        tx_target_yaw = np.int16(self.last_tx_data_prev.get('target_yaw', 0.0))
        tx_roller_speed = np.uint16(self.last_tx_data_prev.get('roller_speed', 0.0))
        tx_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)
        
        # 创建数据帧
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
        
        # 保存状态用于下一次比较
        self.complete_state_prev = self.complete_state
        self.rc_state_prev = self.rc_state
        
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

    def check_comm_timeout(self):
        """检查通信是否超时"""
        current_time = rospy.get_time()
        if current_time - self.last_valid_rx_time > self.comm_timeout:
            rospy.logwarn_throttle(5.0, "Serial communication timeout detected")
            return True
        return False

    def send_heartbeat(self):
        """发送心跳帧保持通信活跃"""
        current_time = rospy.get_time()
        if current_time - self.last_heartbeat_time > self.heartbeat_interval:
            self.last_heartbeat_time = current_time
            
            # 创建心跳帧（空指令）
            heartbeat_data = {
                'distance': 0,
                'target_yaw': 0,
                'roller_speed': 0,
                'yaw': self.yaw_to_target_yaw_angle(self.current_yaw, 0),
                'robot_state': 0x03  # 特殊心跳状态
            }
            
            tx_frame = self.create_tx_frame(heartbeat_data)
            if tx_frame and self.ser and self.ser.is_open:
                try:
                    self.ser.write(tx_frame)
                    rospy.logdebug("Sent heartbeat frame")
                except serial.SerialException:
                    rospy.logwarn("Failed to send heartbeat, possible serial disconnection")

    def run(self):
        """主循环，增强错误恢复能力"""
        buffer = bytearray()
        while not rospy.is_shutdown():
            try:
                # 检查通信超时
                if self.check_comm_timeout():
                    rospy.logwarn("Attempting to recover from communication timeout")
                    if not self.init_serial():
                        rospy.sleep(1.0)
                        continue
                
                # 检查串口状态
                if self.ser is None or not self.ser.is_open:
                    rospy.logwarn("Serial port not open, attempting to reconnect...")
                    if not self.init_serial():
                        rospy.sleep(1.0)
                        continue
                
                # 发送心跳帧
                self.send_heartbeat()
                
                # 尝试读取串口数据
                try:
                    if self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting)
                        if data:
                            buffer.extend(data)
                            rospy.logdebug(f"Received {len(data)} bytes")
                except serial.SerialException as e:
                    rospy.logwarn(f"Serial read error: {e}, attempting to reconnect")
                    if not self.init_serial():
                        rospy.sleep(1.0)
                    continue
                
                # 处理接收到的数据
                processed = False
                while len(buffer) >= self.rx_frame_length:
                    # 查找帧头
                    header_pos = buffer.find(b'\xAA')
                    if header_pos < 0:
                        # 没有找到有效帧头，清空缓冲区
                        buffer.clear()
                        break
                    
                    # 移除头部无效数据
                    if header_pos > 0:
                        rospy.logwarn(f"Discarding {header_pos} bytes before header")
                        del buffer[:header_pos]
                        continue
                    
                    # 检查是否有足够数据
                    if len(buffer) < self.rx_frame_length:
                        break
                    
                    # 提取完整帧
                    frame = bytes(buffer[:self.rx_frame_length])
                    del buffer[:self.rx_frame_length]
                    
                    # 解析数据
                    parsed = self.parse_rx_frame(frame)
                    if parsed:
                        rospy.logdebug(f"Received valid frame: {frame.hex()}")
                        self.publish_wheel_status(parsed)
                        processed = True
                
                # 收到有效数据后发送控制指令
                if processed and self.last_tx_data is not None:
                    tx_frame = self.create_tx_frame(self.last_tx_data)
                    if tx_frame and len(tx_frame) == self.tx_frame_length:
                        try:
                            self.ser.write(tx_frame)
                            rospy.logdebug(f"Sent control frame: {tx_frame.hex()}")
                        except serial.SerialException:
                            rospy.logwarn("Failed to send control frame, possible serial disconnection")
                
                # 控制循环频率
                rospy.sleep(0.001)

            except Exception as e:
                rospy.logerr(f"Unexpected error in main loop: {e}")
                rospy.sleep(0.1)

    def shutdown(self):
        """安全关闭"""
        if self.ser and self.ser.is_open:
            try:
                # 发送停止指令
                stop_data = {
                    'distance': 0,
                    'target_yaw': 0,
                    'roller_speed': 0,
                    'yaw': 0,
                    'robot_state': 0x01  # 停止状态
                }
                tx_frame = self.create_tx_frame(stop_data)
                if tx_frame:
                    self.ser.write(tx_frame)
                    rospy.loginfo("Sent stop command before shutdown")
                
                # 关闭串口
                self.ser.close()
                rospy.loginfo("Serial port closed")
            except Exception as e:
                rospy.logerr(f"Error during shutdown: {e}")

if __name__ == '__main__':
    node = BaseSerialNode()
    rospy.on_shutdown(node.shutdown)
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass