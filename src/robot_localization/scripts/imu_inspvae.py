#!/usr/bin/env python
import rospy
import serial
import struct
from sensor_msgs.msg import Imu
from robot_localization.msg import INSPVAE
import numpy as np
from std_msgs.msg import Header

class IMUParser:
    def __init__(self):
        rospy.init_node('imu_parser_node')
        
        # 参数配置
        self.port = rospy.get_param('~serial_port', '/dev/imu485')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.device_addr = 0x50
        self.rx_frame_length = 7
        self.reconnect_interval = 0.5  # 重连间隔（秒）

        self.ser = None
        self.timer = None

        # 初始尝试连接
        self.init_serial()

        # 发布IMU数据
        self.imu_pub = rospy.Publisher('/inspvae_data', INSPVAE, queue_size=10)

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
            # 连接成功后，启动定时器
            self.start_timer()
            return True
        except serial.SerialException as e:
            rospy.logwarn(f"Failed to connect to {self.port}: {e}. Retrying...")
            self.ser = None
            return False

    def send_query_cmd(self, event):
        """发送查询指令"""
        cmd = bytes.fromhex(f"{self.device_addr:02X} 03 00 3F 00 01 ")
        crc = self.calculate_crc(cmd)
        full_cmd = cmd + crc
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(full_cmd)
        except serial.SerialException as e:
            rospy.logerr(f"Serial write failed: {e}. Triggering reconnection.")
            self.handle_disconnection()

    def parse_response(self, data):
        """解析返回数据"""
        if len(data) != self.rx_frame_length or data[0] != 0x50:
            return None
        
        # CRC校验
        recv_crc = data[-2:]
        calc_crc = self.calculate_crc(data[:-2])
        if recv_crc != calc_crc:
            rospy.logwarn("CRC check failed")
            return None
        
        # 解析角度数据
        yaw_bytes = data[3:5]
        yaw = np.int16(struct.unpack('>h', yaw_bytes)[0]) / 32768.0 * 180.0
        return {'roll': 0, 'pitch': 0, 'yaw': yaw}

    def run(self):
        """主循环，包含断线重连机制."""
        buffer = bytearray()
        while not rospy.is_shutdown():
            # 检查串口是否连接，如果未连接则尝试重连
            if self.ser is None or not self.ser.is_open:
                self.stop_timer() # 确保定时器已停止
                if not self.init_serial():
                    rospy.sleep(self.reconnect_interval)
                    continue

            try:
                # 读取串口数据
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    if data:
                        buffer += data

                # 处理完整帧
                while len(buffer) >= self.rx_frame_length:
                    header_pos = buffer.find(b'\x50')
                    if header_pos == -1:
                        buffer.clear()
                        break
                    
                    if header_pos > 0:
                        buffer = buffer[header_pos:]
                    
                    if len(buffer) < self.rx_frame_length:
                        break
                    
                    frame = buffer[:self.rx_frame_length]
                    buffer = buffer[self.rx_frame_length:]
                    
                    parsed = self.parse_response(frame)
                    if parsed:
                        self.publish_inspvae_data(parsed)
                
                rospy.sleep(0.001)

            except serial.SerialException as e:
                rospy.logerr(f"Serial read error: {e}. Triggering reconnection.")
                self.handle_disconnection()
                rospy.sleep(self.reconnect_interval)
            except Exception as e:
                rospy.logerr(f"An unexpected error occurred in run loop: {e}")
                self.handle_disconnection()
                rospy.sleep(self.reconnect_interval)

    def handle_disconnection(self):
        """处理断开连接的清理工作"""
        self.stop_timer()
        if self.ser:
            try:
                self.ser.close()
            except Exception as e:
                rospy.logerr(f"Error closing serial port: {e}")
        self.ser = None

    def start_timer(self):
        """启动发送指令的定时器"""
        if self.timer is None:
            self.timer = rospy.Timer(rospy.Duration(0.02), self.send_query_cmd)
            rospy.loginfo("Query timer started.")

    def stop_timer(self):
        """停止定时器"""
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None
            rospy.loginfo("Query timer stopped.")

    def publish_inspvae_data(self, angles):
        """发布INSPVAE数据"""
        msg = INSPVAE()
        msg.header = Header(stamp=rospy.Time.now(), frame_id='inspvae')
        msg.yaw = angles['yaw'] % 360
        self.imu_pub.publish(msg)

    @staticmethod
    def calculate_crc(data):
        """Modbus CRC16校验"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

    def shutdown(self):
        """节点关闭时调用"""
        rospy.loginfo("Shutting down IMU parser node.")
        self.stop_timer()
        if self.ser and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    node = IMUParser()
    rospy.on_shutdown(node.shutdown)
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass