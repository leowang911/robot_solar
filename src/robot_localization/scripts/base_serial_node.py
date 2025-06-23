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
import can

class BaseSerialNode:
    def __init__(self):
        rospy.init_node('base_serial_node')

        # 参数配置
        self.frame_part1 = None  # 存储第一次接收到的8字节
        self.frame_part2 = None  # 存储第二次接收到的5字节 共13字节   发14字节

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
        # self.last_tx_data = None
        self.last_tx_data = {
            'distance': 10,
            'target_yaw': 20,
            'roller_speed': 30,
            'yaw':66,
            'robot_state': 1,
            'voltage': 15,
            'error': 0
        }
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
        # self.init_serial()

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

        # 初始化CAN接口
        self.init_can()

    def init_can(self):
        """初始化CAN接口"""
        try:
            self.bus = can.interface.Bus(channel='vcan0', interface='socketcan')
            rospy.loginfo("Connected to CAN interface on channel 'can0'")
        except Exception as e:
            rospy.logerr(f"CAN interface error: {e}")
            rospy.signal_shutdown("CAN interface init failed")
    def inspvae_cb(self, msg):
        # self.latitude = msg.latitude
        # self.longitude = msg.longitude
        self.current_yaw = math.radians(msg.yaw)
    
    def process_data(self, msg):
        # 确保数据是有效的
        if len(msg.data) != 8:
            rospy.logwarn("Invalid frame or wrong data length")
            return None
        if msg.arbitration_id != 0x100 :  # 根据实际的CAN ID进行检查
            rospy.logwarn(f"Unknown CAN frame ID: {hex(msg.arbitration_id)}")
            return None
        # 分别处理第一次和第二次接收到的数据
        if len(msg.data) == 8 and msg.data[0] == 0xAB:
            # print(f"Received second part of frame: {msg.data.hex()}")
        # 第二次消息
            self.frame_part2 = msg.data
        elif len(msg.data) == 8 and msg.data[0] == 0xAA:
            # print(f"Received first part of frame: {msg.data.hex()}")
        # 第一次消息
            self.frame_part1 = msg.data

        
        # 当两部分数据都接收到时，合并它们并开始解析
        if self.frame_part1 and self.frame_part2:
            full_data = self.frame_part1 + self.frame_part2  # 合并两部分数据
            # 重置数据部分
            self.frame_part1 = None
            self.frame_part2 = None
            print(f"Received full data: {full_data.hex()}")
            # 校验和检查  
            # print(f"Checksum: {full_data[12]}, Calculated: {sum(full_data[:12]) & 0xFF}")
            self.parse_can_frame(full_data)
            # if full_data[12] != sum(full_data[:12]) & 0xFF:
            #     rospy.logwarn("Checksum error")
            #     return None
            # else:
               
           




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
            # 这里添加你的紧急停止操作代码（例如：停止电机+发送停止指令等）
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
        """创建发送CAN数据帧"""
        if data is None:
            return None

        state = data.get('robot_state', 0x00)

        if self.complete_state_prev != 0 or state == 0x01:
            self.last_tx_data_prev = data

        tx_distance = int(self.last_tx_data_prev.get('distance', 0.0))
        tx_target_yaw = np.int16(self.last_tx_data_prev.get('target_yaw', 0.0))
        tx_roller_speed = np.uint16(self.last_tx_data_prev.get('roller_speed', 0.0))
        tx_yaw = self.yaw_to_target_yaw_angle(self.current_yaw, 0)

        if self.stop_flag:
            state = 0x01

        # 按照CAN协议将数据分为多个帧  0x55 + 4 dis + 2 yaw + 00
        #                          0x56 + 2 roll + 2 tx_yaw + 1 state + SUM + 00
        frame_data = struct.pack('<BiHBBHHB',
                                 0x55,  # 帧头
                                 tx_distance,
                                 tx_target_yaw & 0xFFFF,
                                 0x00,
                                 0x56,  # 第二帧头
                                 tx_roller_speed & 0xFFFF,
                                 tx_yaw & 0xFFFF,
                                 state)

        # 计算校验和
        checksum = sum(frame_data[:13]) & 0xFF
        frame_data = frame_data + bytes([checksum])

        # 如果数据超出8字节，拆分成多个帧
        frames = []
        max_data_length = 8  # CAN帧最大字节数

        for i in range(0, len(frame_data), max_data_length):
            frame_chunk = frame_data[i:i+max_data_length]

            # 判断如果拆分后的数据长度不足8字节，补充0x00
            if len(frame_chunk) < max_data_length:
                frame_chunk = frame_chunk.ljust(max_data_length, b'\x00')  # 使用0x00补齐

            frames.append(frame_chunk)  # 保存数据部分

        self.complete_state_prev = self.complete_state
        self.rc_state_prev = self.rc_state

        return frames  # 返回多个分帧数据

    
    def parse_can_frame(self, msg):
        """解析CAN数据帧"""
        try:
            # if msg.arbitration_id == 0x123:  # 根据帧ID解析
            if msg[0] == 0xAA and msg[8] == 0xAB:  
                # 根据帧ID解析   AA +2 speed +4 dis +1 sensor
                                # AB + 00 + 4 + 00 + 00      
                self.speed = struct.unpack('>h', msg[1:3])[0]
                self.distance = struct.unpack('>i', msg[3:7])[0]
                self.sensor_state = msg[7]
                #第二条开始为0xAB+0x00

                self.complete_state = msg[10]
                self.rc_state = msg[11]
                self.voltage = msg[12]
                self.error = msg[13]
    #         self.speed = struct.unpack('>h', data[1:3])[0]
    #         self.distance = struct.unpack('>i', data[3:7])[0]


                print(f"Parsed CAN frame: speed={self.speed}, \
                        distance={self.distance}, sensor_state={self.sensor_state}, \
                        complete_state={self.complete_state}, rc_state={self.rc_state},\
                        voltage={self.voltage}, error={self.error}")
                return {
                    'speed': self.speed,
                    'distance': self.distance,
                    'sensor_state': self.sensor_state,
                    'complete_state': self.complete_state,
                    'rc_state': self.rc_state,
                    'voltage': self.voltage,
                    'error': self.error
                }
            else:
                rospy.logwarn(f"Unknown CAN frame ID: {msg.arbitration_id}")
                return None
        except Exception as e:
            rospy.logerr(f"Error parsing CAN frame: {e}")
            return None

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

        
    #测试发送CAN数据帧
    def send_can_frame(self, data):
        """发送CAN数据帧"""
        try:
            frames = self.create_tx_frame(data)  # 获取分帧数据
            if frames:
                for frame in frames:  # 遍历每个帧
                    msg = can.Message(
                        arbitration_id=0x101,  # 设置CAN ID
                        data=frame,  # 数据部分
                        is_extended_id=False  # 使用标准帧
                    )
                    self.bus.send(msg)  # 发送单个CAN帧
                    rospy.loginfo(f"Sent: {msg}")
        except Exception as e:
            rospy.logerr(f"Error sending CAN frame: {e}")

    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            try:
                # 接收CAN帧
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    parsed = self.process_data(msg)  # 传递完整的 CAN 消息对象
                    if parsed:
                        self.publish_wheel_status(parsed)

                # 发送CAN帧
                # if self.last_tx_data:
                # if 1:
                #     self.send_can_frame(self.last_tx_data)

                rospy.sleep(0.01)
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