#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json

# 回调函数，处理接收到的消息
def gps_callback(msg):
    try:
        # 将JSON格式的字符串解析为Python字典
        gps_data = json.loads(msg.data)
        
        # 输出GPS数据为8位浮点数
        # 从gps_data中提取latitude和longitude
        latitude_str = gps_data.get("latitude", "N/A")
        longitude_str = gps_data.get("longitude", "N/A")

        # 确保将其转换为浮动类型，如果是有效值（不是"N/A"），否则使用默认值0.0
        latitude_drone = float(latitude_str) if latitude_str != "N/A" else 0.0
        longitude_drone = float(longitude_str) if longitude_str != "N/A" else 0.0

        # 保留8位小数，确保数据格式正确
        latitude_drone = round(latitude_drone, 8)
        longitude_drone = round(longitude_drone, 8)

        # 输出保留8位小数的经纬度
        rospy.loginfo("Latitude: %.8f, Longitude: %.8f, Yaw_drone: %.8f", latitude_drone, longitude_drone, yaw_drone)

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
