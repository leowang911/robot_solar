# test_serial_minimal.py
# -*- coding: utf-8 -*-
import serial
import time

port = '/dev/ttyUSB1'
baudrate = 460800

try:
    ser = serial.Serial(port, baudrate, timeout=1, rtscts=False)
    # print(f"成功打开串口")
    print("1")
    for _ in range(10):
        if ser.in_waiting > 0:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            print("2")
        else:
            print("等待数据...")
        time.sleep(0.1)
            
except Exception as e:
    print("错误")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()