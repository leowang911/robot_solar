#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json

# 回调函数，处理接收到的消息
def gps_callback(msg):
    try:
        # 将JSON格式的字符串解析为Python字典
        gps_data = json.loads(msg.data)
        
        # 输出GPS数据
        latitude = gps_data.get("latitude", "N/A")
        longitude = gps_data.get("longitude", "N/A")
        heading = gps_data.get("heading", "N/A")
        pitch = gps_data.get("pitch", "N/A")
        rospy.loginfo("Latitude: %s, Longitude: %s, Heading: %s, Pitch: %s", latitude, longitude, heading, pitch)

    except json.JSONDecodeError:
        rospy.logerr("Received invalid JSON data")

def gps_listener():
    # 初始化ROS节点
    rospy.init_node('gps_listener', anonymous=True)
    
    # 订阅/gps/raw话题
    rospy.Subscriber('/gps/raw', String, gps_callback)
    
    # 保持程序运行，直到节点关闭
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_listener()
    except rospy.ROSInterruptException:
        pass
