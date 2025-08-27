#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import math
from robot_localization.msg import baseStatus,INSPVAE  # 根据实际包名调整
from robot_control.msg import controlData  # 根据实际包名调整
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

        # 初始化串口
        self.ser = None
        self.init_serial() # 初始尝试连接

        # 发布接收数据
        self.wheel_pub = rospy.Publisher('base_status', baseStatus, queue_size=10)

        # 订阅控制指令
        rospy.Subscriber('/control_data', controlData, self.control_data_callback)
        rospy.Subscriber('/inspvae_data',INSPVAE,self.inspvae_cb)

        self.service_stop = rospy.Service(
        '/emergency_stop',  # 服务名称（必须与客户端一致）
        Trigger,             # 服务类型
        self.handle_emergency_stop # 处理函数
        )

        self.service_reboot = rospy.Service(
        '/reboot',  # 服务名称（必须与客户端一致）
        Trigger,             # 服务类型
        self.handle_reboot # 处理函数
        )

    def inspvae_cb(self, msg):
        # self.latitude = msg.latitude
        # self.longitude = msg.longitude
        self.current_yaw = math.radians(msg.yaw)
    

    def init_serial(self):
        """初始化或重新初始化串口连接."""
        if self.ser and self.ser.is_open:
            self.ser.close()
        
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            rospy.loginfo(f"Successfully connected to serial port {self.port}")
            return True
        except serial.SerialException as e:
            rospy.logwarn(f"Failed to connect to {self.port}: {e}. Retrying...")
            self.ser = None
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
        # rospy.logwarning(f'yaw: {self.current_yaw}')

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
        
    def yaw_to_target_yaw_angle(self, yaw, current_yaw):
        """将航向角转换为控制角度"""
        # rospy.loginfo(f"current_yaw: {self.current_yaw}")
        # imu ccw and cw !!!!!! 记得根据实际情况修改 九洲需要加-
        angle= self.angle_dir*(math.degrees(yaw)*100) + math.degrees(current_yaw)*100
        #计算gps距离
        # rospy.loginfo(f"angle: {angle}")
        if angle > 36000:
            angle -= 36000
        if angle < 0:
            angle += 36000
        # rospy.loginfo(f"angle: {angle}")
        return np.uint16(angle)
    
    def handle_emergency_stop(self,req):
        """
        当收到紧急停止服务请求时，执行此回调函数
        :param req: Trigger 请求（无字段）
        :return: TriggerResponse 包含执行结果和消息
        """
        try:
            # 这里添加你的紧急停止操作代码（例如：停止电机、发送停止指令等）
            # control = self.compose_control(0,0,self.current_yaw,0,1)
            # self.control_pub(control)
            
            self.stop_flag = True
            time.sleep(0.1)
            # -----------------------------------------------------------------
            rospy.loginfo("Executing emergency stop...")
            # -----------------------------------------------------------------
            # 返回成功响应
            return TriggerResponse(
                success=True,
                message="Emergency stop executed successfully"
            )
        except Exception as e:
            rospy.logerr(f"Emergency stop failed: {str(e)}")
            return TriggerResponse(
                success=False,
                message=f"Error during emergency stop: {str(e)}"
            )
        
    def handle_reboot(self,req):
        """
        当收到重启服务请求时，执行此回调函数
        :param req: Trigger 请求（无字段）
        :return: TriggerResponse 包含执行结果和消息
        """
        try:
            # 这里添加你的重启操作代码
            # control = self.compose_control(0,0,self.current_yaw,0,1)
            # self.control_pub(control)
            self.stop_flag = False
            # -----------------------------------------------------------------
            rospy.loginfo("Reboot")
            # -----------------------------------------------------------------
            # 返回成功响应
            return TriggerResponse(
                success=True,
                message="Reboot executed successfully"
            )
        except Exception as e:
            rospy.logerr(f"Reboot failed: {str(e)}")
            return TriggerResponse(
                success=False,
                message=f"Error during reboot: {str(e)}"
            )

    def create_tx_frame(self, data):
        """创建发送数据帧"""
        if data is None:
            return None

        state = data.get('robot_state', 0x00)

        # if self.rc_state_prev != 2 and self.rc_state == 2:
        #     self.last_tx_data_prev = data
        #     state = 0x01

        if self.complete_state != 0 or state == 0x01:
            self.last_tx_data_prev = data

        # if state == 0x02 and self.complete_state_prev == 0 and self.complete_state ==1:
        #     self.last_tx_data_prev = data
        #     # rospy.loginfo("robot_state: 0x02, complete_state_prev: 0, complete_state: 1")
        #     state = 0x01
 
        tx_distance = int(self.last_tx_data_prev.get('distance', 0.0))
        tx_target_yaw = np.int16(self.last_tx_data_prev.get('target_yaw', 0.0))
        tx_roller_speed = np.uint16(self.last_tx_data_prev.get('roller_speed', 0.0))
        # tx_yaw = np.uint16(data.get('yaw', 0.0))
        tx_yaw = self.yaw_to_target_yaw_angle(self.current_yaw,0)
        #self.yaw_to_target_yaw_angle(self.current_yaw,0)
        # rospy.logwarn(f"tx_yaw: {tx_yaw}")
        # rospy.logwarn(f"tx_distance: {tx_distance}, tx_target_yaw: {tx_target_yaw}, tx_roller_speed: {tx_roller_speed}, tx_yaw: {tx_yaw}, state: {state}")

        # if state == 0x02 and self.complete_state_prev ==0 and self.complete_state == 1:
        #     state = 0x01

        if self.stop_flag:
            state = 0x01
        
 
        # if state == 0x02 & self.complete_state == 0:
        #     tx_distance = self.distance_prev
        #     tx_target_yaw = self.yaw_prev


        frame = struct.pack('<BiHHHB',
                            0x55,
                            tx_distance,    
                            tx_target_yaw & 0xFFFF,
                            # 2800 & 0xFFFF,  # 2200
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
        """主循环，包含断线重连机制."""
        buffer = bytearray()
        while not rospy.is_shutdown():
            # 检查串口是否连接，如果未连接则尝试重连
            if self.ser is None or not self.ser.is_open:
                if not self.init_serial():
                    rospy.sleep(1.0)  # 等待1秒后重试
                    continue

            try:
                # 1. 尝试读取串口数据
                if self.ser.in_waiting > 0:
                    buffer += self.ser.read(self.ser.in_waiting)

                # 2. 处理接收到的完整帧
                if len(buffer) >= self.rx_frame_length:
                    header_pos = buffer.find(b'\xAA')
                    if header_pos != -1 and len(buffer) >= header_pos + self.rx_frame_length:
                        frame = buffer[header_pos:header_pos + self.rx_frame_length]
                        buffer = buffer[header_pos + self.rx_frame_length:]

                        parsed = self.parse_rx_frame(frame)
                        if parsed:
                            rospy.loginfo(f"Received frame: {frame.hex()}")
                            self.publish_wheel_status(parsed)

                            # 3. 收到有效帧后发送数据
                            if self.last_tx_data is not None:
                                tx_frame = self.create_tx_frame(self.last_tx_data)
                                if tx_frame and len(tx_frame) == self.tx_frame_length:
                                    self.ser.write(tx_frame)
                                    # rospy.logdebug(f"Sent frame: {tx_frame.hex()}")
                    elif header_pos == -1:
                        # 如果找不到帧头，清空缓冲区以防数据错乱
                        buffer = bytearray()


                rospy.sleep(0.001)

            except serial.SerialException as e:
                rospy.logerr(f"Serial communication error: {e}. Disconnecting and will try to reconnect.")
                if self.ser:
                    self.ser.close()
                self.ser = None
                rospy.sleep(0.5)  # 等待0.5秒后尝试重连
            except Exception as e:
                rospy.logerr(f"An unexpected error occurred in run loop: {e}")
                rospy.sleep(1.0)


    def shutdown(self):
        """安全关闭"""
        rospy.loginfo("Shutting down serial node.")
        if self.ser and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    node = BaseSerialNode()
    rospy.on_shutdown(node.shutdown)
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass