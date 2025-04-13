#!/usr/bin/env python3
import rospy
import json
import paho.mqtt.client as mqtt
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String, Float32

class MQTTRobotPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('mqtt_robot_publisher', anonymous=True)
        rate = rospy.Rate(0.2)
        
        # MQTT配置参数（从ROS参数服务器读取或使用默认值）
        self.mqtt_broker = rospy.get_param('~mqtt_broker', '36.7.136.5')
        self.mqtt_port = rospy.get_param('~mqtt_port', 13234)
        self.mqtt_topic = rospy.get_param('~mqtt_topic', 'robot/status')
        
        # 初始化MQTT客户端
        self.mqtt_client = mqtt.Client(client_id="robot_publisher")
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
        
        # 订阅ROS话题
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/task/start', Bool, self.task_callback)
        rospy.Subscriber('/mission/route_id', String, self.route_callback)
        rospy.Subscriber('/battery/voltage', Float32, self.battery_callback)
        
        # 存储最新数据
        self.robot_data = {
            "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
            "gps": {"latitude": 0.0, "longitude": 0.0},
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            "task_started": False,
            "route_id": "unknown",
            "battery_voltage": 0.0
        }
    

    def _connect_mqtt(self):
        """连接MQTT服务器并启动循环"""
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            rospy.loginfo(f"Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as e:
            rospy.logerr(f"MQTT connection failed: {str(e)}")
            rospy.signal_shutdown("MQTT connection error")

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            rospy.loginfo("MQTT connected successfully")
            client.subscribe(self.mqtt_topic)
        else:
            rospy.logerr(f"MQTT connection failed with code {rc}")

    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT消息接收回调"""
        try:
            # 解析消息（假设为JSON格式）
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            
            # 转换为ROS消息并发布
            ros_msg = String()
            ros_msg.data = payload
            self.ros_pub.publish(ros_msg)
            rospy.loginfo(f"Received MQTT message: {payload}")
        except Exception as e:
            rospy.logwarn(f"Failed to process MQTT message: {str(e)}")

    def _on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        rospy.logwarn(f"MQTT disconnected, attempting reconnect... (rc={rc})")
        self._connect_mqtt()  # 自动重连

    def shutdown(self):
        """节点关闭时清理资源"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        rospy.loginfo("MQTT client disconnected")


    # ROS回调函数
    def imu_callback(self, msg):
        self.robot_data["acceleration"] = {
            "x": msg.linear_acceleration.x,
            "y": msg.linear_acceleration.y,
            "z": msg.linear_acceleration.z
        }
        self.robot_data["angular_velocity"] = {
            "x": msg.angular_velocity.x,
            "y": msg.angular_velocity.y,
            "z": msg.angular_velocity.z
        }
        
    def gps_callback(self, msg):
        self.robot_data["gps"] = {
            "latitude": msg.latitude,
            "longitude": msg.longitude
        }
        
    def task_callback(self, msg):
        self.robot_data["task_started"] = msg.data
        
    def route_callback(self, msg):
        self.robot_data["route_id"] = msg.data
        
    def battery_callback(self, msg):
        self.robot_data["battery_voltage"] = msg.data
        
    # 主循环
    def run(self):
        rate = rospy.Rate(1)  # 1Hz发送频率
        while not rospy.is_shutdown():
            try:
                # 发布数据到MQTT
                payload = json.dumps(self.robot_data)
                self.mqtt_client.publish(self.mqtt_topic, payload,qos=1)
                rospy.loginfo("Data sent to MQTT: %s", payload)
            except Exception as e:
                rospy.logerr("MQTT Publish Error: %s", str(e))
            rate.sleep()
        
        # 关闭连接
        self.mqtt_client.disconnect()

if __name__ == '__main__':
    node = MQTTRobotPublisher()
    node.run()
