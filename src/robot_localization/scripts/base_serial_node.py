#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from robot_localization.msg import baseStatus  # 根据实际包名调整
from robot_control.msg import controlData  # 根据实际包名调整
from std_msgs.msg import Header

class BaseSerialNode:
    def __init__(self):
        rospy.init_node('base_serial_node')

        # 参数配置
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.rx_frame_length = 13       # 接收帧长度
        self.tx_frame_length = 13       # 发送帧长度
        self.lock = threading.Lock()    # 串口访问锁
        self.send_queue = []            # 发送队列

        self.speed = 0
        self.distance = 0
        self.sensor_state = 0
        self.complete_state = 0
        self.rc_state = 0


        # 初始化串口
        self.ser = None
        self.init_serial()

        # 发布接收数据
        self.wheel_pub = rospy.Publisher('base_status', baseStatus, queue_size=10)

        # 订阅控制指令
        rospy.Subscriber('/control_data', controlData, self.control_data_callback)
        

        # 启动处理线程
        self.running = True
        self.recv_thread = threading.Thread(target=self.recv_loop)
        self.send_thread = threading.Thread(target=self.send_loop)
        self.recv_thread.start()
        self.send_thread.start()

    def init_serial(self):
        """初始化串口连接"""
        try:
            with self.lock:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1
                )
                rospy.loginfo(f"Connected to {self.port} at {self.baudrate} baud")
        except serial.SerialException as e:
            rospy.logerr(f"Serial port error: {e}")
            rospy.signal_shutdown("Serial port init failed")

    def control_data_callback(self, msg):
        """速度指令回调"""
        tx_data = {
            'distance': msg.distance,
            'target_yaw': msg.target_yaw,
            'roller_speed': msg.roller_speed,
            'yaw': msg.yaw,
            'state': msg.robot_state
        }
        self.queue_tx_data(tx_data)


    def queue_tx_data(self, data):
        """将发送数据加入队列"""
        with self.lock:
            self.send_queue.append(data)
            # 保持队列最新状态，丢弃旧数据
            if len(self.send_queue) > 5:
                self.send_queue.pop(0)

    def parse_rx_frame(self, data):
        """解析接收数据帧"""
        try:
            if data[0] != 0xAA  or len(data) != self.rx_frame_length or data[12] != sum(data[:12]) & 0xFF:
                # rospy.loginfo("Invalid frame or checksum error")
                return None
            
            # 解析各字段（大端序）
            self.speed = struct.unpack('>h', bytes(data[1:3]))[0]
            self.distance = struct.unpack('>i', bytes(data[3:7]))[0]
            self.sensor_state = data[7]
            self.complete_state = data[9]
            self.rc_state = data[10]

            return {
                'speed': self.speed,
                'distance': self.distance,
                'sensor_state': self.sensor_state,
                'complete_state': self.complete_state,
                'rc_state': self.rc_state
            }

        except Exception as e:
            rospy.logerr(f"Parse error: {e}")
            return None

    def create_tx_frame(self, data):
        """创建发送数据帧"""
        scaled_distance = int(data.get('distance', 0.0) )
        scaled_target_yaw = int(data.get('target_yaw', 0.0) * 100)
        scaled_roller_speed = int(data.get('roller_speed', 0.0) * 100)
        scaled_yaw = int(data.get('yaw', 0.0) * 100)

        state = data.get('state', 0x00)

        if(data.state == 0x02 &
           self.complete_state == 1):
            state = 0x01

        frame = struct.pack('<BihhhB',
                            0x55,
                            scaled_distance & 0xFFFFFFFF,
                            scaled_target_yaw & 0xFFFF,
                            scaled_roller_speed & 0xFFFF,
                            scaled_yaw & 0xFFFF,
                            state,
                        #   0x0A00, #distance mm
                        #   0x00A0, #target_yaw degree*100
                        #   0x00A0, #roller speed degree 0-28*100
                        #   0x00A0,  # current yaw*100
                        #   0x02, 
                            )   

        # 计算校验和
        checksum = sum(frame[:12]) & 0xFF
        return frame + bytes([checksum])

    def recv_loop(self):
        """接收数据处理循环"""
        buffer = bytearray()
        while self.running and not rospy.is_shutdown():
            try:
                with self.lock:
                    if self.ser.in_waiting > 0:
                        buffer += self.ser.read(self.ser.in_waiting)

                # 处理完整帧
                while len(buffer) >= self.rx_frame_length:
                    # 查找帧头
                    header_pos = buffer.find(b'\xAA')
                    if header_pos == -1:
                        buffer.clear()
                        break

                    # 提取完整帧
                    frame = buffer[header_pos:header_pos+self.rx_frame_length]
                    buffer = buffer[header_pos+self.rx_frame_length:]

                    # 解析并发布数据
                    parsed = self.parse_rx_frame(frame)
                    if parsed:
                        self.publish_wheel_status(parsed)

            except Exception as e:
                rospy.logerr(f"Recv loop error: {e}")
                rospy.sleep(1)

    def send_loop(self):
        """发送数据处理循环"""
        while self.running and not rospy.is_shutdown():
            try:
                if not self.send_queue:
                    rospy.sleep(0.01)
                    continue

                # 获取待发送数据
                with self.lock:
                    data = self.send_queue.pop(0)

                # 生成数据帧
                frame = self.create_tx_frame(data)
                # if self.complete_state == 0x01:
                #     frame[12] = 0x00

                if len(frame) != self.tx_frame_length:
                    continue

                # 发送数据
                with self.lock:
                    # if(self.complete_state != 0x00):
                    #     self.ser.write(frame)
                    self.ser.write(frame)
                    rospy.logdebug(f"Sent: {frame.hex()}")

            except Exception as e:
                rospy.logerr(f"Send loop error: {e}")
                rospy.sleep(1)

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
        self.wheel_pub.publish(msg)

    def shutdown(self):
        """安全关闭"""
        self.running = False
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        self.recv_thread.join()
        self.send_thread.join()

if __name__ == '__main__':
    node = BaseSerialNode()
    rospy.on_shutdown(node.shutdown)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass