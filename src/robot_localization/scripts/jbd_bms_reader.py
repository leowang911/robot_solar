#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import struct
from sensor_msgs.msg import BatteryState

def checksum(data):
    """
    计算JBD BMS协议的校验和。
    """
    return sum(data) & 0xFFFF

def main():
    rospy.init_node('jbd_bms_reader')

    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baudrate = rospy.get_param('~baudrate', 9600)

    pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
    except serial.SerialException as e:
        rospy.logerr("Error opening serial port {}: {}".format(port, e))
        return

    rate = rospy.Rate(2) # 2 Hz

    # 读取基本信息的命令
    read_basic_info_cmd = bytearray([0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77])

    while not rospy.is_shutdown():
        try:
            ser.write(read_basic_info_cmd)
            response = ser.read(31) # 响应长度为31字节

            if len(response) == 31 and response[0] == 0xDD and response[1] == 0x03 and response[-1] == 0x77:
                # 验证校验和
                # 红色为被校验字节，为所有的字节的总和；后面 2 个为校验结果，为前面所有校验的总和取反+1 的结果
                # 校验和计算
                # chksum = sum(bytearray(response[2:-3]))
                # if chksum != struct.unpack('>H', response[-3:-1])[0]:
                #     rospy.logwarn("Checksum mismatch")
                #     continue

                data = struct.unpack('>H H H H H H H H B B B B H H H H H', response[4:-3])

                battery_state_msg = BatteryState()
                battery_state_msg.header.stamp = rospy.Time.now()
                battery_state_msg.voltage = data[0] / 100.0
                
                # 电流为负数时处理
                current = data[1]
                if current & 0x8000:
                    current = -(0xFFFF - current + 1)
                battery_state_msg.current = current / 100.0
                
                battery_state_msg.percentage = data[12] / 100.0
                battery_state_msg.capacity = data[3] / 1000.0 # 标称容量
                battery_state_msg.charge = data[2] / 1000.0 # 剩余容量
                
                battery_state_msg.present = True
                
                # 根据电流判断电源状态
                if battery_state_msg.current > 0:
                    battery_state_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                elif battery_state_msg.current < 0:
                    battery_state_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                else:
                    battery_state_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

                pub.publish(battery_state_msg)

        except serial.SerialException as e:
            rospy.logerr("Serial communication error: {}".format(e))
        except Exception as e:
            rospy.logerr("An error occurred: {}".format(e))

        rate.sleep()

    ser.close()

if __name__ == '__main__':
    main()
